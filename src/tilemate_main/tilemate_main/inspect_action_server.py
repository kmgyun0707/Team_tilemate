#!/usr/bin/env python3
import copy
import json
import threading
import requests
import time

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
        self.declare_parameter("local_result_path", "inspect_result.json")

        self.web_result_url = self.get_parameter("web_result_url").value
        self.local_result_path = self.get_parameter("local_result_path").value

        from tilemate_main.onrobot import RG
        self.gripper = RG(
            gripper_cfg.GRIPPER_NAME,
            gripper_cfg.TOOLCHARGER_IP,
            gripper_cfg.TOOLCHARGER_PORT,
        )

        # -----------------------------
        # callback groups 분리
        # -----------------------------
        self.server_cb_group = ReentrantCallbackGroup()
        self.pattern_client_cb_group = ReentrantCallbackGroup()

        # -----------------------------
        # action server
        # -----------------------------
        self._action_server = ActionServer(
            self,
            Inspect,
            "tile/inspect",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.server_cb_group,
        )

        # -----------------------------
        # nested action client
        # namespace 절대경로로 고정
        # -----------------------------
        self.pattern_action_name = f"/{robot_cfg.robot_id}/tile/pattern_inspect"
        self._pattern_action_client = ActionClient(
            self,
            PatternInspect,
            self.pattern_action_name,
            callback_group=self.pattern_client_cb_group,
        )

        self.inspection_engine = WallTileInspectionEngine(self)

        self.initialize_robot()
        self.get_logger().info(
            f"\033[94m [2/3] [DEPTH_INSPECT] initialize Done! pattern_action={self.pattern_action_name}\033[0m"
        )

    def initialize_robot(self):
        from DSR_ROBOT2 import (
            set_tool,
            set_tcp,
            ROBOT_MODE_MANUAL,
            ROBOT_MODE_AUTONOMOUS,
            set_robot_mode,
            wait,
        )

        set_robot_mode(ROBOT_MODE_MANUAL)
        set_tool(self.robot_cfg.tool)
        set_tcp(self.robot_cfg.tcp)
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        wait(1.0)

    def move_to_inspect_pose(self):
        from DSR_ROBOT2 import posx, movesx, mwait

        candidates = [
            posx([380.6733093261719, 177.2272491455078, 179.8480987548828, 89.89385223388672, 91.91939544677734, 92.74739837646484]),
            posx([380.705, 157.182, 139.804, 90.000, 90.001, 89.999]),
            posx([380.705, 127.182, 109.804, 90.000, 90.001, 89.999]),
        ]

        try:
            ret_move = movesx(candidates, time=5.0)
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

    def call_pattern_inspect_action_blocking(self, timeout_sec: float = 60.0):
        """
        PatternInspect 액션을 동기적으로 호출한 것처럼 기다리고
        { pattern_name: anomaly_score } dict를 반환한다.
        실패 시 None 반환.
        """
        self.get_logger().info(
            f"[INSPECT] waiting for pattern inspect action server... name={self.pattern_action_name}"
        )

        ok = self._pattern_action_client.wait_for_server(timeout_sec=3.0)
        if not ok:
            self.get_logger().warn("[INSPECT] pattern inspect action server not available")
            return None

        goal_msg = PatternInspect.Goal()
        goal_msg.target_frames = 60
        goal_msg.pixel_threshold = 60
        goal_msg.match_dist_threshold = 50.0

        self.get_logger().info(
            "[INSPECT] sending goal to pattern inspect action "
            f"(target_frames={goal_msg.target_frames}, "
            f"pixel_threshold={goal_msg.pixel_threshold}, "
            f"match_dist_threshold={goal_msg.match_dist_threshold})"
        )

        done_event = threading.Event()
        result_holder = {
            "accepted": False,
            "status": None,
            "result": None,
            "exception": None,
        }

        def _result_done_cb(result_future):
            try:
                result_wrap = result_future.result()
                result_holder["status"] = getattr(result_wrap, "status", None)
                result_holder["result"] = getattr(result_wrap, "result", None)

                self.get_logger().info(
                    f"[INSPECT] pattern result callback fired: status={result_holder['status']}"
                )
            except Exception as e:
                result_holder["exception"] = e
                self.get_logger().warn(f"[INSPECT] pattern result callback exception: {e}")
            finally:
                done_event.set()

        def _goal_done_cb(goal_future):
            try:
                goal_handle = goal_future.result()

                if goal_handle is None:
                    result_holder["exception"] = RuntimeError("goal_handle is None")
                    done_event.set()
                    return

                result_holder["accepted"] = bool(goal_handle.accepted)
                self.get_logger().info(
                    f"[INSPECT] pattern goal response received: accepted={goal_handle.accepted}"
                )

                if not goal_handle.accepted:
                    done_event.set()
                    return

                result_future = goal_handle.get_result_async()
                result_future.add_done_callback(_result_done_cb)

            except Exception as e:
                result_holder["exception"] = e
                self.get_logger().warn(f"[INSPECT] pattern goal callback exception: {e}")
                done_event.set()

        send_goal_future = self._pattern_action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(_goal_done_cb)

        start_t = time.time()
        while rclpy.ok():
            if done_event.wait(timeout=0.1):
                break

            elapsed = time.time() - start_t
            if elapsed > timeout_sec:
                self.get_logger().warn("[INSPECT] pattern inspect action timeout")
                return None

        if result_holder["exception"] is not None:
            self.get_logger().warn(
                f"[INSPECT] pattern inspect action failed: {result_holder['exception']}"
            )
            return None

        if not result_holder["accepted"]:
            self.get_logger().warn("[INSPECT] pattern inspect goal rejected")
            return None

        pattern_result = result_holder["result"]
        if pattern_result is None:
            self.get_logger().warn("[INSPECT] pattern inspect result is None")
            return None

        self.get_logger().info(
            f"[INSPECT] pattern result success={pattern_result.success}, "
            f"message={pattern_result.message}, "
            f"count={len(pattern_result.pattern_scores)}"
        )

        if not pattern_result.success:
            self.get_logger().warn(
                f"[INSPECT] pattern inspect reported failure: {pattern_result.message}"
            )
            return None

        pattern_scores_map = {}
        for item in pattern_result.pattern_scores:
            try:
                name = str(item.pattern_name).strip()
                if not name:
                    continue
                score = float(item.anomaly_score)
                pattern_scores_map[name] = score
                self.get_logger().info(
                    f"[INSPECT] parsed pattern score: name={name}, score={score}"
                )
            except Exception as e:
                self.get_logger().warn(f"[INSPECT] failed to parse pattern score item: {e}")

        self.get_logger().info(f"[INSPECT] pattern result map: {pattern_scores_map}")
        return pattern_scores_map
        
    async def execute_callback(self, goal_handle):
        result = Inspect.Result()

        try:
            self.get_logger().info("[INSPECT] start inspection action")

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = "inspect_canceled_before_start"
                return result

            self.publish_feedback(goal_handle, 1, 0.1, "open_gripper")
            self.gripper.open_gripper()

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = "inspect_canceled_after_open_gripper"
                return result

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
            # 1) 단차 검사
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
            # 2) 패턴 검사
            # --------------------------------------------------
            self.publish_feedback(goal_handle, 4, 0.75, "pattern_inspect")
            self.get_logger().info("[INSPECT] start pattern inspect action")
            pattern_scores = self.call_pattern_inspect_action_blocking()
            self.get_logger().info(f"[INSPECT] pattern inspect finished: {pattern_scores}")

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

            # --------------------------------------------------
            # 3) anomaly_score만 payload에 merge
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
            # 4) 내부/로컬 저장
            # --------------------------------------------------
            self._save_result_payload(result_dict)

            self.publish_feedback(goal_handle, 6, 0.9, "save_internal_and_local")
            ok_local, local_msg = self.save_result_to_local(result_dict)
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
            # 5) 웹에 JSON 전송
            # --------------------------------------------------
            self.publish_feedback(goal_handle, 7, 0.95, "send_result_to_web")
            ok_web, web_msg = self.send_result_to_web(result_dict)

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
            # 6) 최종 성공
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