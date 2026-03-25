#!/usr/bin/env python3
import json
import time
import rclpy
import DR_init

from pathlib import Path

from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from tilemate_msgs.action import Press
from tilemate_main.robot_config import RobotConfig, GripperConfig

class PressActionServer(Node):
    def __init__(self, robot_cfg: RobotConfig, gripper_cfg: GripperConfig, boot_node: Node):
        super().__init__("press_action_server", namespace=robot_cfg.robot_id)

        self._boot_node = boot_node
        self.robot_cfg = robot_cfg
        self.gripper_cfg = gripper_cfg

        self.cb_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            Press,
            "tile/press",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group,
        )

        self.declare_parameter("default_result_json_path", "inspect_result.json")
        self.declare_parameter("default_press_threshold_mm", 3.0)
        self.declare_parameter("default_approach_offset_mm", 20.0)
        self.declare_parameter("default_press_overshoot_mm", 1.0)
        self.declare_parameter("default_press_force_n", 15.0)
        self.declare_parameter("default_press_speed_mm_s", 10.0)
        self.declare_parameter("wall_push_direction_sign", 1)

        from tilemate_main.onrobot import RG
        self.gripper = RG(
            gripper_cfg.GRIPPER_NAME,
            gripper_cfg.TOOLCHARGER_IP,
            gripper_cfg.TOOLCHARGER_PORT,
        )

        self.initialize_robot()
        self.get_logger().info("\033[94m [5/5] [PRESS] initialize Done!\033[0m")

    # ----------------------------
    # Action callbacks
    # ----------------------------
    def goal_callback(self, goal_request):
        if goal_request.result_json_path is None:
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().warn("Cancel request received")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        result = Press.Result()

        try:
            ok, pressed_count, depth, message = self.perform_task_once(goal_handle)

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.pressed_count = int(pressed_count)
                result.pressed_depth = float(depth)
                result.message = "canceled"
                return result

            if ok:
                goal_handle.succeed()
            else:
                goal_handle.abort()

            result.success = bool(ok)
            result.pressed_count = int(pressed_count)
            result.pressed_depth = float(depth)
            result.message = str(message)
            return result

        except Exception as e:
            self.get_logger().error(f"Execution failed: {e}")
            try:
                goal_handle.abort()
            except Exception:
                pass
            result.success = False
            result.pressed_count = 0
            result.pressed_depth = 0.0
            result.message = f"exception:{e}"
            return result

    # ----------------------------
    # Robot init
    # ----------------------------
    def initialize_robot(self):
        from DSR_ROBOT2 import (
            set_tool,
            set_tcp,
            ROBOT_MODE_MANUAL,
            ROBOT_MODE_AUTONOMOUS,
            set_robot_mode,
        )

        self.get_logger().info("[PRESS] initialize_robot()")

        set_robot_mode(ROBOT_MODE_MANUAL)
        set_tool(self.robot_cfg.tool)
        set_tcp(self.robot_cfg.tcp)
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)

        time.sleep(1.0)

    # ----------------------------
    # Helpers
    # ----------------------------
    def check_abort(self, goal_handle) -> bool:
        return bool(goal_handle.is_cancel_requested)

    def sleep_interruptible(self, sec: float, goal_handle, dt: float = 0.05) -> bool:
        from DSR_ROBOT2 import wait

        t0 = time.time()
        while (time.time() - t0) < float(sec):
            if self.check_abort(goal_handle):
                return False
            wait(float(dt))
        return True

    def disable_compliance(self):
        from DSR_ROBOT2 import release_force, release_compliance_ctrl

        try:
            release_force()
        except Exception:
            pass

        try:
            release_compliance_ctrl()
        except Exception:
            pass

    def read_ft_guess(self):
        """
        환경마다 force/wrench 함수명이 달라서 후보를 순차 탐색.
        반환: (fx, fy, fz, tx, ty, tz) 또는 None
        """
        import DSR_ROBOT2 as dr

        candidates = [
            "get_external_force",
            "get_ext_force",
            "get_tool_force",
            "get_current_force",
            "get_external_wrench",
            "get_tool_wrench",
        ]

        for name in candidates:
            fn = getattr(dr, name, None)
            if callable(fn):
                try:
                    v = fn()

                    if (
                        isinstance(v, (list, tuple))
                        and len(v) == 2
                        and isinstance(v[0], (list, tuple))
                    ):
                        v = v[0]

                    if isinstance(v, (list, tuple)) and len(v) >= 6:
                        return tuple(float(x) for x in v[:6])
                except Exception:
                    pass

        return None

    def publish_feedback(
        self,
        goal_handle,
        step: int,
        state: str,
        total_defects: int = 0,
        current_defect_index: int = 0,
        progress: float = 0.0,
    ):
        fb = Press.Feedback()
        fb.step = int(step)
        fb.progress = float(progress)
        fb.state = str(state)
        fb.total_defects = int(total_defects)
        fb.current_defect_index = int(current_defect_index)
        goal_handle.publish_feedback(fb)

    def load_json_result(self, path_str: str):
        path = Path(path_str)
        if not path.exists():
            raise FileNotFoundError(f"json not found: {path}")
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)

    def resolve_goal_value(self, req):
        result_json_path = req.result_json_path.strip()
        if result_json_path == "":
            result_json_path = str(self.get_parameter("default_result_json_path").value).strip()

        if result_json_path == "":
            raise ValueError("result_json_path is empty")

        press_threshold_mm = (
            float(req.press_threshold_mm)
            if float(req.press_threshold_mm) > 0.0
            else float(self.get_parameter("default_press_threshold_mm").value)
        )

        approach_offset_mm = (
            float(req.approach_offset_mm)
            if float(req.approach_offset_mm) > 0.0
            else float(self.get_parameter("default_approach_offset_mm").value)
        )

        press_overshoot_mm = (
            float(req.press_overshoot_mm)
            if float(req.press_overshoot_mm) > 0.0
            else float(self.get_parameter("default_press_overshoot_mm").value)
        )

        press_force_n = (
            float(req.press_force_n)
            if float(req.press_force_n) > 0.0
            else float(self.get_parameter("default_press_force_n").value)
        )

        press_speed_mm_s = (
            float(req.press_speed_mm_s)
            if float(req.press_speed_mm_s) > 0.0
            else float(self.get_parameter("default_press_speed_mm_s").value)
        )

        return (
            result_json_path,
            press_threshold_mm,
            approach_offset_mm,
            press_overshoot_mm,
            press_force_n,
            press_speed_mm_s,
        )

    def extract_defect_tiles(self, data, press_threshold_mm: float):
        tiles = data.get("tiles", [])
        if not isinstance(tiles, list):
            return []

        targets = []
        for tile in tiles:
            if not isinstance(tile, dict):
                continue

            center = tile.get("base_center_mm", None)
            if center is None or not isinstance(center, (list, tuple)) or len(center) < 3:
                continue

            targets.append(tile)

        return targets

    def build_press_pose_from_tile(self, tile: dict, approach_offset_mm: float):
        center = tile.get("base_center_mm", None)
        if center is None or len(center) < 3:
            raise ValueError(f"tile '{tile.get('name', '?')}' has no valid base_center_mm")

        cx = float(center[0])
        cy = float(center[1])
        cz = float(center[2])

        sign = int(self.get_parameter("wall_push_direction_sign").value)
        if sign not in (-1, 1):
            sign = 1

        pre_contact_pose = [cx, cy - sign * float(approach_offset_mm), cz+60.0, 90.0, 90.0, 90.0]
        contact_pose = [cx, cy, cz+60.0, 90.0, 90.0, 90.0]

        return pre_contact_pose, contact_pose

    # ----------------------------
    # press
    # ----------------------------
    def compact_tile(
        self,
        goal_handle,
        pre_contact_pose,
        contact_pose,
        press_force_n: float,
        press_speed_mm_s: float,
        press_depth_mm: float,
        timeout_s: float = 10.0,
        log_dt: float = 0.2,
    ):
        from DSR_ROBOT2 import (
            set_ref_coord,
            task_compliance_ctrl,
            set_desired_force,
            check_force_condition,
            get_current_posx,
            movel,
            posx,
            wait,
            release_force,
            mwait,
            DR_FC_MOD_REL,
            DR_AXIS_Y,
            DR_BASE,
            DR_MV_MOD_REL,
        )

        sign = int(self.get_parameter("wall_push_direction_sign").value)
        if sign not in (-1, 1):
            sign = 1

        self.get_logger().info(
            f"[PRESS] start | "
            f"force={press_force_n:.3f}, "
            f"speed={press_speed_mm_s:.3f}, "
            f"depth={press_depth_mm:.3f}, "
            f"pre_contact_pose={pre_contact_pose}"
        )

        if self.check_abort(goal_handle):
            return (False, 0.0, "canceled")

        set_ref_coord(DR_BASE)

        self.gripper.close_gripper()

        # 1) 누를 곳 앞까지 접근
        movel(posx(pre_contact_pose), ref=DR_BASE, vel=50, acc=50)
        mwait()

        # 2) Y 방향만 순응
        task_compliance_ctrl(
            stx=[3000, 20, 3000, 300, 300, 300],
            time=0.0
        )
        wait(0.2)

        # 3) Y 방향 힘 제어
        set_desired_force(
            fd=[0, 30, 0, 0, 0, 0],
            dir=[0, 1, 0, 0, 0, 0],
            mod=DR_FC_MOD_REL
        )

        t0 = time.time()
        last_log = 0.0
        touched = False
        contact_y = 0.0

        try:
            while (time.time() - t0) < float(timeout_s):
                if self.check_abort(goal_handle):
                    return (False, 0.0, "canceled")

                now = time.time()
                ft = self.read_ft_guess()

                if (now - last_log) >= float(log_dt):
                    if ft:
                        self.get_logger().info(
                            f"[FT][SEARCH] Fx={ft[0]:7.2f} Fy={ft[1]:7.2f} Fz={ft[2]:7.2f} | "
                            f"Tx={ft[3]:7.2f} Ty={ft[4]:7.2f} Tz={ft[5]:7.2f}"
                        )
                    last_log = now

                cond = check_force_condition(DR_AXIS_Y, min=0, max=float(press_force_n), ref=DR_BASE)
                if cond == -1 or cond is True:
                    cur_pos, _ = get_current_posx(DR_BASE)
                    contact_y = float(cur_pos[1])
                    touched = True

                    self.get_logger().info(
                        f"[CONTACT] touched=True contact_y={contact_y:.3f}"
                    )

                    release_force()
                    break

                wait(0.1)

            if not touched:
                self.get_logger().warn("[CONTACT] failed (timeout)")
                return (True, 0.0, "contact_failed")

            # 4) 추가 누르기
            dy = sign * float(press_depth_mm)
            movel(
                posx([0.0, dy, 0.0, 0.0, 0.0, 0.0]),
                ref=DR_BASE,
                vel=float(press_speed_mm_s),
                acc=float(press_speed_mm_s),
                mod=DR_MV_MOD_REL,
            )
            mwait()

            final_pos, _ = get_current_posx(DR_BASE)
            final_y = float(final_pos[1])
            depth = abs(final_y - contact_y)

            self.get_logger().info(f"[RESULT] final_pose={final_pos} depth={depth:.3f}")

            self.disable_compliance()
            # 5) 복귀
            movel(posx(pre_contact_pose), ref=DR_BASE, vel=50, acc=50)
            mwait()

            return (True, depth, "press_success")

        finally:
            self.disable_compliance()
            try:
                wait(0.1)
            except Exception:
                pass

    # ----------------------------
    # Task sequence
    # ----------------------------
    def perform_task_once(self, goal_handle):
        req = goal_handle.request

        self.publish_feedback(goal_handle, 1, "load_json", 0, 0, 10.0)

        (
            result_json_path,
            press_threshold_mm,
            approach_offset_mm,
            press_overshoot_mm,
            press_force_n,
            press_speed_mm_s,
        ) = self.resolve_goal_value(req)

        data = self.load_json_result(result_json_path)

        self.publish_feedback(goal_handle, 2, "parse_targets", 0, 0, 20.0)

        targets = self.extract_defect_tiles(data, press_threshold_mm)
        total_defects = len(targets)

        self.get_logger().info(f"[PRESS] total targets={total_defects}")

        if total_defects <= 0:
            return (True, 0, 0.0, "no press target")

        pressed_count = 0
        last_depth = 0.0

        for idx, tile in enumerate(targets, start=1):
            if self.check_abort(goal_handle):
                return (False, pressed_count, last_depth, "canceled")

            tile_name = str(tile.get("name", f"tile_{idx}"))
            press_depth_mm = float(press_overshoot_mm)

            self.publish_feedback(
                goal_handle,
                3,
                f"approach_{tile_name}",
                total_defects,
                idx,
                20.0 + (60.0 * float(idx - 1) / float(total_defects)),
            )

            pre_contact_pose, contact_pose = self.build_press_pose_from_tile(
                tile,
                approach_offset_mm,
            )

            self.get_logger().info(
                f"[PRESS] target={tile_name} "
                f"press_depth_mm={press_depth_mm:.3f} "
                f"pre_contact_pose={pre_contact_pose} "
                f"contact_pose={contact_pose}"
            )

            self.publish_feedback(
                goal_handle,
                4,
                f"press_{tile_name}",
                total_defects,
                idx,
                40.0 + (40.0 * float(idx - 1) / float(total_defects)),
            )

            ok, depth, message = self.compact_tile(
                goal_handle,
                pre_contact_pose=pre_contact_pose,
                contact_pose=contact_pose,
                press_force_n=press_force_n,
                press_speed_mm_s=press_speed_mm_s,
                press_depth_mm=press_depth_mm,
                timeout_s=20.0,
                log_dt=0.1,
            )

            last_depth = float(depth)

            if not ok:
                return (False, pressed_count, last_depth, f"{tile_name}:{message}")

            pressed_count += 1

            self.publish_feedback(
                goal_handle,
                5,
                f"return_{tile_name}",
                total_defects,
                idx,
                90.0,
            )
        return (True, pressed_count, last_depth, "press_success")

def main(args=None):
    rclpy.init(args=args)

    robot_cfg = RobotConfig()
    gripper_cfg = GripperConfig()

    boot = rclpy.create_node("dsr_boot_press_tile", namespace=robot_cfg.robot_id)

    DR_init.__dsr__id = robot_cfg.robot_id
    DR_init.__dsr__model = robot_cfg.robot_model
    DR_init.__dsr__node = boot

    import DSR_ROBOT2  # noqa: F401

    node = PressActionServer(robot_cfg, gripper_cfg, boot)

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

        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()