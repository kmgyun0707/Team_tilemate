#!/usr/bin/env python3
import copy
import json
import threading
import time
import requests

import rclpy
import DR_init

from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from action_msgs.msg import GoalStatus

from tilemate_msgs.action import Inspect, PatternInspect
from tilemate_main.robot_config import RobotConfig, GripperConfig
from tilemate_main.wall_tile_inspection_engine import WallTileInspectionEngine


class PatternInspectClientHelper(Node):
    """
    pattern_inspect action client를 별도 노드/별도 executor thread에서 처리해서
    inspect action server 내부 nested action deadlock을 피하기 위한 helper
    """
    def __init__(self, robot_id: str):
        super().__init__("pattern_inspect_client_helper", namespace=robot_id)

        self._cb_group = ReentrantCallbackGroup()
        self._action_name = f"/{robot_id}/tile/pattern_inspect"

        self._client = ActionClient(
            self,
            PatternInspect,
            self._action_name,
            callback_group=self._cb_group,
        )

        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self)

        self._spin_thread = threading.Thread(
            target=self._executor.spin,
            daemon=True,
        )
        self._spin_thread.start()

        self.get_logger().info(f"[PATTERN_HELPER] ready: action={self._action_name}")

    def shutdown_helper(self):
        self.get_logger().info("[PATTERN_HELPER] shutdown start")

        try:
            self._executor.remove_node(self)
        except Exception:
            pass

        try:
            self.destroy_node()
        except Exception:
            pass

        self.get_logger().info("[PATTERN_HELPER] shutdown done")

    def call(self,
             target_frames: int = 60,
             pixel_threshold: int = 60,
             match_dist_threshold: float = 50.0,
             timeout_sec: float = 90.0):
        """
        pattern inspect action을 helper thread에서 호출하고
        {pattern_name: anomaly_score} dict 반환
        실패 시 None 반환
        """
        self.get_logger().info(
            f"[PATTERN_HELPER] wait_for_server: {self._action_name}"
        )

        ok = self._client.wait_for_server(timeout_sec=3.0)
        if not ok:
            self.get_logger().warn("[PATTERN_HELPER] pattern action server not available")
            return None

        goal_msg = PatternInspect.Goal()
        goal_msg.target_frames = int(target_frames)
        goal_msg.pixel_threshold = int(pixel_threshold)
        goal_msg.match_dist_threshold = float(match_dist_threshold)

        self.get_logger().info(
            "[PATTERN_HELPER] send_goal "
            f"(target_frames={goal_msg.target_frames}, "
            f"pixel_threshold={goal_msg.pixel_threshold}, "
            f"match_dist_threshold={goal_msg.match_dist_threshold})"
        )

        done_event = threading.Event()
        holder = {
            "accepted": False,
            "status": None,
            "result": None,
            "exception": None,
        }

        def _result_done_cb(fut):
            try:
                wrap = fut.result()
                holder["status"] = getattr(wrap, "status", None)
                holder["result"] = getattr(wrap, "result", None)

                self.get_logger().info(
                    f"[PATTERN_HELPER] result callback fired: status={holder['status']}"
                )
            except Exception as e:
                holder["exception"] = e
                self.get_logger().error(
                    f"[PATTERN_HELPER] result callback exception: {e}"
                )
            finally:
                done_event.set()

        def _goal_done_cb(fut):
            try:
                goal_handle = fut.result()

                if goal_handle is None:
                    holder["exception"] = RuntimeError("goal_handle is None")
                    done_event.set()
                    return

                holder["accepted"] = bool(goal_handle.accepted)

                self.get_logger().info(
                    f"[PATTERN_HELPER] goal response: accepted={goal_handle.accepted}"
                )

                if not goal_handle.accepted:
                    done_event.set()
                    return

                result_future = goal_handle.get_result_async()
                result_future.add_done_callback(_result_done_cb)

            except Exception as e:
                holder["exception"] = e
                self.get_logger().error(
                    f"[PATTERN_HELPER] goal callback exception: {e}"
                )
                done_event.set()

        send_goal_future = self._client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(_goal_done_cb)

        start_t = time.time()
        while rclpy.ok():
            if done_event.wait(timeout=0.1):
                break

            if (time.time() - start_t) > timeout_sec:
                self.get_logger().warn("[PATTERN_HELPER] timeout")
                return None

        if holder["exception"] is not None:
            self.get_logger().error(
                f"[PATTERN_HELPER] failed: {holder['exception']}"
            )
            return None

        if not holder["accepted"]:
            self.get_logger().warn("[PATTERN_HELPER] goal rejected")
            return None

        if holder["status"] != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(
                f"[PATTERN_HELPER] action failed: status={holder['status']}"
            )
            return None

        result = holder["result"]
        if result is None:
            self.get_logger().warn("[PATTERN_HELPER] result is None")
            return None

        self.get_logger().info(
            f"[PATTERN_HELPER] result success={result.success}, "
            f"message={result.message}, count={len(result.pattern_scores)}"
        )

        if not result.success:
            self.get_logger().warn(
                f"[PATTERN_HELPER] result failure: {result.message}"
            )
            return None

        out = {}
        for item in result.pattern_scores:
            try:
                name = str(item.pattern_name).strip()
                if not name:
                    continue
                out[name] = float(item.anomaly_score)
                self.get_logger().info(
                    f"[PATTERN_HELPER] parsed: name={name}, score={out[name]}"
                )
            except Exception as e:
                self.get_logger().warn(
                    f"[PATTERN_HELPER] parse failed: {e}"
                )

        self.get_logger().info(f"[PATTERN_HELPER] result map: {out}")
        return out


class InspectActionServer(Node):

    def __init__(self, robot_cfg: RobotConfig, gripper_cfg: GripperConfig, boot_node: Node):
        super().__init__("inspect_action_server", namespace=robot_cfg.robot_id)

        self._boot_node = boot_node
        self.robot_cfg = robot_cfg
        self.gripper_cfg = gripper_cfg

        self._result_lock = threading.Lock()
        self._last_result_payload = None
        self._result_history = []

        self.declare_parameter("web_result_url", "http://192.168.10.22:8000/api/inspect/result")
        # self.declare_parameter("web_result_url", "http://192.168.10.39:8000/api/inspect/result")

        self.declare_parameter("local_result_path", "inspect_result.json")

        self.web_result_url = self.get_parameter("web_result_url").value
        self.local_result_path = self.get_parameter("local_result_path").value

        # lazy init
        self.gripper = None
        self._gripper_initialized = False
        self._robot_initialized = False

        # callback group
        self.server_cb_group = ReentrantCallbackGroup()

        # action server
        self._action_server = ActionServer(
            self,
            Inspect,
            "tile/inspect",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.server_cb_group,
        )

        # inspection engine
        self.inspection_engine = WallTileInspectionEngine(self)

        # pattern helper (별도 노드/별도 executor)
        self.pattern_helper = PatternInspectClientHelper(robot_cfg.robot_id)

        self.get_logger().info("\033[94m [2/3] [INSPECT] initialize Done!\033[0m")

    def ensure_gripper_initialized(self):
        if self._gripper_initialized and self.gripper is not None:
            return True, "already_initialized"

        try:
            from tilemate_main.onrobot import RG

            self.get_logger().info("[INSPECT] gripper init start")
            self.gripper = RG(
                self.gripper_cfg.GRIPPER_NAME,
                self.gripper_cfg.TOOLCHARGER_IP,
                self.gripper_cfg.TOOLCHARGER_PORT,
            )
            self._gripper_initialized = True
            self.get_logger().info("[INSPECT] gripper init done")
            return True, "gripper_initialized"

        except Exception as e:
            self.get_logger().error(f"[INSPECT] gripper init failed: {e}")
            return False, str(e)

    def ensure_robot_initialized(self):
        if self._robot_initialized:
            return True, "already_initialized"

        try:
            from DSR_ROBOT2 import (
                set_tool,
                set_tcp,
                ROBOT_MODE_MANUAL,
                ROBOT_MODE_AUTONOMOUS,
                set_robot_mode,
                wait,
            )

            self.get_logger().info("[INSPECT] robot init start")
            set_robot_mode(ROBOT_MODE_MANUAL)
            set_tool(self.robot_cfg.tool)
            set_tcp(self.robot_cfg.tcp)
            set_robot_mode(ROBOT_MODE_AUTONOMOUS)
            wait(1.0)

            self._robot_initialized = True
            self.get_logger().info("[INSPECT] robot init done")
            return True, "robot_initialized"

        except Exception as e:
            self.get_logger().error(f"[INSPECT] robot init failed: {e}")
            return False, str(e)

    def move_to_inspect_pose(self):
        from DSR_ROBOT2 import posx, movesx, mwait

        candidates = [
            posx([380.6733093261719, 177.2272491455078, 179.8480987548828, 89.89385223388672, 91.91939544677734, 92.74739837646484]),
            posx([380.705, 157.182, 139.804, 90.000, 90.001, 89.999]),
            posx([380.705, 127.182, 109.804, 90.000, 90.001, 89.999]),
        ]

        try:
            ret_move = movesx(candidates, time=8.0)
            ret_wait = mwait()
            self.get_logger().info(f"[INSPECT] movesx ret={ret_move}, mwait ret={ret_wait}")

            if isinstance(ret_wait, (int, float)) and ret_wait < 0:
                return False, f"mwait_failed:{ret_wait}"

            return True, "move_done"

        except Exception as e:
            return False, f"move_exception:{e}"

    def _save_result_payload(self, payload: dict):
        with self._result_lock:
            self._last_result_payload = copy.deepcopy(payload)
            self._result_history.append(copy.deepcopy(payload))

            if len(self._result_history) > 50:
                self._result_history.pop(0)

    def get_last_result_payload(self):
        with self._result_lock:
            if self._last_result_payload is None:
                return None
            return copy.deepcopy(self._last_result_payload)

    def load_last_result_from_file(self):
        path = self.local_result_path
        try:
            with open(path, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception as e:
            self.get_logger().warn(f"[INSPECT] failed to load local result file: {e}")
            return None

    def save_result_to_local(self, payload: dict):
        path = self.local_result_path
        if not path:
            return False, "local_result_path is empty"

        try:
            with self._result_lock:
                with open(path, "w", encoding="utf-8") as f:
                    json.dump(payload, f, indent=2, ensure_ascii=False)
            return True, path
        except Exception as e:
            return False, str(e)

    def send_result_to_web(self, payload: dict):
        url = self.web_result_url

        if not url:
            return False, "web_result_url is empty"

        try:
            r = requests.post(url, json=payload, timeout=10)
            r.raise_for_status()
            return True, r.text
        except Exception as e:
            return False, str(e)

    def publish_feedback(self, goal_handle, step: int, progress: float, state: str):
        feedback = Inspect.Feedback()
        feedback.step = int(step)
        feedback.progress = float(progress)
        feedback.state = str(state)
        goal_handle.publish_feedback(feedback)

        self.get_logger().info(
            f"[INSPECT][FEEDBACK] step={feedback.step}, progress={feedback.progress:.1f}, state={feedback.state}"
        )

    def goal_callback(self, goal_request):
        self.get_logger().info("[INSPECT] goal request accepted")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().warn("[INSPECT] cancel request received")
        return CancelResponse.ACCEPT

    def call_pattern_inspect_action(self):
        self.get_logger().info("[INSPECT] start pattern helper call")

        result = self.pattern_helper.call(
            target_frames=60,
            pixel_threshold=60,
            match_dist_threshold=50.0,
            timeout_sec=90.0,
        )

        self.get_logger().info(f"[INSPECT] pattern helper finished: {result}")
        return result

    async def execute_callback(self, goal_handle):
        result = Inspect.Result()

        try:
            self.get_logger().info("[INSPECT] start inspection action")

            ok, msg = self.ensure_gripper_initialized()
            if not ok:
                goal_handle.abort()
                result.success = False
                result.message = f"gripper_init_failed:{msg}"
                return result

            ok, msg = self.ensure_robot_initialized()
            if not ok:
                goal_handle.abort()
                result.success = False
                result.message = f"robot_init_failed:{msg}"
                return result

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = "inspect_canceled_before_start"
                return result

            # --------------------------------------------------
            # 1) gripper open
            # --------------------------------------------------
            self.publish_feedback(goal_handle, 1, 0.1, "open_gripper")
            self.gripper.open_gripper()

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = "inspect_canceled_after_open_gripper"
                return result

            # --------------------------------------------------
            # 2) move inspect pose
            # --------------------------------------------------
            self.publish_feedback(goal_handle, 2, 0.3, "move_to_inspect_pose")
            ok_move, move_msg = self.move_to_inspect_pose()
            if not ok_move:
                goal_handle.abort()
                result.success = False
                result.message = f"inspect_move_failed:{move_msg}"
                return result

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = "inspect_canceled_after_move"
                return result

            # --------------------------------------------------
            # 3) depth analyze
            # --------------------------------------------------
            self.publish_feedback(goal_handle, 3, 0.6, "depth_analyze")

            robot_posx = [380.705, 127.182, 109.804, 90.000, 90.001, 89.999]
            inspect_result = self.inspection_engine.analyze_once(robot_posx=robot_posx)

            if not isinstance(inspect_result, dict):
                goal_handle.abort()
                result.success = False
                result.message = "depth_inspect_failed:invalid_result_type"
                return result

            depth_success = bool(inspect_result.get("success", False))
            depth_message = str(inspect_result.get("message", ""))

            self.get_logger().info(
                f"[INSPECT] depth analyze done: success={depth_success}, message={depth_message}"
            )

            if not depth_success:
                goal_handle.abort()
                result.success = False
                result.message = f"depth_inspect_failed:{depth_message}"
                return result

            result_dict = inspect_result.get("result_dict")
            if result_dict is None:
                goal_handle.abort()
                result.success = False
                result.message = "depth_inspect_failed:result_dict_missing"
                return result

            self.get_logger().info(
                f"[INSPECT] depth payload created: tile_count={len(result_dict.get('tiles', []))}"
            )

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = "inspect_canceled_after_depth_analysis"
                return result

            # --------------------------------------------------
            # 4) pattern inspect
            # --------------------------------------------------
            self.publish_feedback(goal_handle, 4, 0.75, "pattern_inspect")
            pattern_scores = self.call_pattern_inspect_action()

            if pattern_scores is None:
                goal_handle.abort()
                result.success = False
                result.message = "pattern_inspect_failed:call_failed"
                return result

            if len(pattern_scores) == 0:
                goal_handle.abort()
                result.success = False
                result.message = "pattern_inspect_failed:empty_result"
                return result

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = "inspect_canceled_after_pattern_analysis"
                return result

            # --------------------------------------------------
            # 5) merge anomaly_score
            # --------------------------------------------------
            self.publish_feedback(goal_handle, 5, 0.85, "merge_pattern_scores")

            merged_count = 0
            for tile in result_dict.get("tiles", []):
                tile_name = str(tile.get("name", "")).strip()
                if tile_name in pattern_scores:
                    tile["anomaly_score"] = float(pattern_scores[tile_name])
                    merged_count += 1
                else:
                    tile["anomaly_score"] = 0.0

            self.get_logger().info(
                f"[INSPECT] merge complete: merged_count={merged_count}, "
                f"total_tiles={len(result_dict.get('tiles', []))}"
            )

            if merged_count == 0:
                goal_handle.abort()
                result.success = False
                result.message = "pattern_inspect_failed:no_matching_tile_names"
                return result

            self.get_logger().info(f"[INSPECT] merged payload: {result_dict}")

            # --------------------------------------------------
            # 6) internal/local save
            # --------------------------------------------------
            self.get_logger().info("[INSPECT][DBG] before internal save")
            self._save_result_payload(result_dict)
            self.get_logger().info("[INSPECT][DBG] after internal save")

            self.publish_feedback(goal_handle, 6, 0.9, "save_internal_and_local")

            self.get_logger().info("[INSPECT][DBG] before local save")
            ok_local, local_msg = self.save_result_to_local(result_dict)
            self.get_logger().info("[INSPECT][DBG] after local save")

            if ok_local:
                self.get_logger().info(f"[INSPECT] local save success: {local_msg}")
            else:
                self.get_logger().warn(f"[INSPECT] local save failed: {local_msg}")

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = "inspect_canceled_after_merge"
                return result

            # --------------------------------------------------
            # 7) web send
            # --------------------------------------------------
            self.publish_feedback(goal_handle, 7, 0.95, "send_result_to_web")

            self.get_logger().info("[INSPECT][DBG] before web send")
            ok_web, web_msg = self.send_result_to_web(result_dict)
            self.get_logger().info("[INSPECT][DBG] after web send")

            if ok_web:
                self.get_logger().info("[INSPECT] web send success")
                if ok_local:
                    final_message = "inspect_complete"
                else:
                    final_message = f"inspect_complete_but_local_save_failed:{local_msg}"
            else:
                self.get_logger().warn(f"[INSPECT] web send failed: {web_msg}")
                if ok_local:
                    final_message = f"inspect_complete_but_web_send_failed:{web_msg}"
                else:
                    final_message = (
                        f"inspect_complete_but_local_save_failed:{local_msg}"
                        f"_and_web_send_failed:{web_msg}"
                    )

            # --------------------------------------------------
            # 8) done
            # --------------------------------------------------
            self.publish_feedback(goal_handle, 8, 1.0, "done")
            goal_handle.succeed()

            result.success = True
            result.message = final_message
            self.get_logger().info(f"[INSPECT] result success=True, message={final_message}")
            return result

        except Exception as e:
            self.get_logger().error(f"[INSPECT] failed: {e}")
            try:
                goal_handle.abort()
            except Exception:
                pass
            result.success = False
            result.message = f"exception:{e}"
            return result


def main(args=None):
    rclpy.init(args=args)

    robot_cfg = RobotConfig()
    gripper_cfg = GripperConfig()

    boot = rclpy.create_node("dsr_boot_inspect", namespace=robot_cfg.robot_id)

    DR_init.__dsr__node = boot
    DR_init.__dsr__id = robot_cfg.robot_id
    DR_init.__dsr__model = robot_cfg.robot_model

    import DSR_ROBOT2  # noqa: F401

    node = InspectActionServer(robot_cfg, gripper_cfg, boot)

    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    ex.add_node(boot)

    try:
        ex.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
    finally:
        try:
            if hasattr(node, "pattern_helper") and node.pattern_helper is not None:
                node.pattern_helper.shutdown_helper()
        except Exception:
            pass

        try:
            ex.remove_node(node)
        except Exception:
            pass

        try:
            ex.remove_node(boot)
        except Exception:
            pass

        try:
            node.destroy_node()
        except Exception:
            pass

        try:
            boot.destroy_node()
        except Exception:
            pass

        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()