#!/usr/bin/env python3
import copy
import json
import threading
import time
import requests
import rclpy
import DR_init

from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from tilemate_msgs.action import Inspect
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

        self.declare_parameter("web_result_url", "http://192.168.10.48:8000/api/inspect/result")
        self.declare_parameter("local_result_path", "inspect_result.json")

        self.web_result_url = self.get_parameter("web_result_url").value
        self.local_result_path = self.get_parameter("local_result_path").value

        from tilemate_main.onrobot import RG
        self.gripper = RG(
            gripper_cfg.GRIPPER_NAME,
            gripper_cfg.TOOLCHARGER_IP,
            gripper_cfg.TOOLCHARGER_PORT,
        )

        self.cb_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            Inspect,
            "tile/inspect",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group,
        )

        self.inspection_engine = WallTileInspectionEngine(self)

        self.initialize_robot()
        self.get_logger().info("\033[94m [5/6] [INSPECT] initialize Done!\033[0m")

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

        movesx(candidates, time= 5.0)
        mwait()

    def _save_result_payload(self, payload: dict):
        with self._result_lock:
            self._last_result_payload = copy.deepcopy(payload)
            self._result_history.append(copy.deepcopy(payload))

            # 필요 이상 커지지 않게 최근 50개만 유지
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
            r = requests.post(url, json=payload, timeout=3)
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

    async def execute_callback(self, goal_handle):
        result = Inspect.Result()

        try:
            self.get_logger().info("[INSPECT] start inspection action")

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = "inspect_canceled_before_start"
                return result

            self.publish_feedback(goal_handle, 1, 10.0, "open_gripper")
            self.gripper.open_gripper()

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = "inspect_canceled_after_open_gripper"
                return result

            self.publish_feedback(goal_handle, 2, 30.0, "move_to_inspect_pose")
            self.move_to_inspect_pose()

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = "inspect_canceled_after_move"
                return result

            self.publish_feedback(goal_handle, 3, 60.0, "analyze")
            robot_posx = [380.705, 127.182, 109.804, 90.000, 90.001, 89.999]
            inspect_result = self.inspection_engine.analyze_once(robot_posx=robot_posx)

            success = bool(inspect_result.get("success", False))
            message = str(inspect_result.get("message", ""))

            if success:
                self.publish_feedback(goal_handle, 4, 75.0, "save_internal_and_local")

                result_dict = inspect_result.get("result_dict", None)
                self.get_logger().info(f"[INSPECT] result_dict: {result_dict}")

                if result_dict is None:
                    self.get_logger().warn("[INSPECT] result_dict is missing")
                    goal_handle.succeed()
                    result.success = True
                    result.message = "inspect_complete_but_result_dict_missing"
                    return result

                # 1) 내부 메모리 저장
                self._save_result_payload(result_dict)

                # 2) 로컬 JSON 파일 덮어쓰기 저장
                ok_local, local_msg = self.save_result_to_local(result_dict)
                if ok_local:
                    self.get_logger().info(f"[INSPECT] local save success: {local_msg}")
                else:
                    self.get_logger().warn(f"[INSPECT] local save failed: {local_msg}")

                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = "inspect_canceled_after_analysis"
                    return result

                # 3) 웹 전송
                self.publish_feedback(goal_handle, 5, 90.0, "send_result_to_web")
                ok_web, web_msg = self.send_result_to_web(result_dict)

                if ok_web:
                    self.get_logger().info("[INSPECT] web send success")
                    if ok_local:
                        message = "inspect_complete"
                    else:
                        message = f"inspect_complete_but_local_save_failed:{local_msg}"
                else:
                    self.get_logger().warn(f"[INSPECT] web send failed: {web_msg}")
                    if ok_local:
                        message = f"inspect_complete_but_web_send_failed:{web_msg}"
                    else:
                        message = f"inspect_complete_but_local_save_failed:{local_msg}_and_web_send_failed:{web_msg}"

            self.publish_feedback(goal_handle, 6, 100.0, "done")
            goal_handle.succeed()

            result.success = success
            result.message = message
            self.get_logger().info(f"[INSPECT] result success={result.success}, message={result.message}")
            return result

        except Exception as e:
            self.get_logger().error(f"[INSPECT] failed: {e}")
            goal_handle.abort()
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

    import DSR_ROBOT2  # noqa

    node = InspectActionServer(robot_cfg, gripper_cfg, boot)

    ex = MultiThreadedExecutor()
    ex.add_node(node)

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
            node.destroy_node()
            boot.destroy_node()
        except Exception:
            pass

        rclpy.shutdown()


if __name__ == "__main__":
    main()