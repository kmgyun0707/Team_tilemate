#!/usr/bin/env python3
import time
import rclpy
import DR_init

from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor

from tilemate_msgs.action import PlaceTile


# ----------------------------
# 로봇 설정 상수
# ----------------------------
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"

VELOCITY = 40
ACC = 60

# ----------------------------
# 압착 축 설정
# ----------------------------
# 벽 방향으로 누르는 축이 BASE 기준 X/Y/Z 중 무엇인지 지정
PRESS_AXIS_INDEX = 2   # 0:X, 1:Y, 2:Z
PRESS_AXIS_SIGN = -1.0
# pressed_depth = (current_axis - contact_axis) * PRESS_AXIS_SIGN
# 눌릴수록 양수 증가하도록 sign 조정


class PlaceTileActionServer(Node):
    def __init__(self, boot_node: Node):
        super().__init__("place_tile_action_server", namespace=ROBOT_ID)

        self._boot_node = boot_node
        self.cb_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            PlaceTile,
            "tile/place_press",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group,

        )

        # 중요:
        # onrobot.py 안에 DSR_ROBOT2 top-level import가 있으면 안 됨.
        from tilemate_main.onrobot import RG
        self.gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)

        self.initialize_robot()
        self.get_logger().info("PlaceTileActionServer ready.")

    # ----------------------------
    # Action callbacks
    # ----------------------------
    def goal_callback(self, goal_request):
        if goal_request.placement_index < 0:
            self.get_logger().warn("Reject goal: placement_index < 0")
            return GoalResponse.REJECT

        if goal_request.max_press_force <= 0.0:
            self.get_logger().warn("Reject goal: max_press_force <= 0")
            return GoalResponse.REJECT

        if goal_request.target_press_depth <= 0.0:
            self.get_logger().warn("Reject goal: target_press_depth <= 0")
            return GoalResponse.REJECT

        self.get_logger().info(
            f"Goal accepted: placement_index={goal_request.placement_index}, "
            f"max_press_force={goal_request.max_press_force:.3f}, "
            f"target_press_depth={goal_request.target_press_depth:.3f}"
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().warn("Cancel request received")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        result = PlaceTile.Result()

        try:
            ok, depth, message = self.perform_task_once(goal_handle)

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.press_depth = float(depth)
                result.message = "canceled"
                return result

            if ok:
                goal_handle.succeed()
            else:
                goal_handle.abort()

            result.success = bool(ok)
            result.press_depth = float(depth)
            result.message = str(message)
            return result

        except Exception as e:
            self.get_logger().error(f"Execution failed: {e}")
            result.success = False
            result.press_depth = 0.0
            result.message = f"exception:{e}"
            return result

    # ----------------------------
    # Robot init
    # ----------------------------
    def initialize_robot(self):
        from DSR_ROBOT2 import (
            set_tool,
            set_tcp,
            get_tool,
            get_tcp,
            ROBOT_MODE_MANUAL,
            ROBOT_MODE_AUTONOMOUS,
            get_robot_mode,
            set_robot_mode,
        )

        self.get_logger().info("[PLACE_TILE] initialize_robot()")

        set_robot_mode(ROBOT_MODE_MANUAL)
        set_tool(ROBOT_TOOL)
        set_tcp(ROBOT_TCP)
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)

        time.sleep(0.2)

        self.get_logger().info("#" * 50)
        self.get_logger().info(f"ROBOT_ID: {ROBOT_ID}")
        self.get_logger().info(f"ROBOT_MODEL: {ROBOT_MODEL}")
        self.get_logger().info(f"ROBOT_TCP: {get_tcp()}")
        self.get_logger().info(f"ROBOT_TOOL: {get_tool()}")
        self.get_logger().info(f"ROBOT_MODE: {get_robot_mode()}")
        self.get_logger().info(f"VELOCITY: {VELOCITY}")
        self.get_logger().info(f"ACC: {ACC}")
        self.get_logger().info("#" * 50)

    # ----------------------------
    # Helpers
    # ----------------------------
    def check_abort(self, goal_handle) -> bool:
        return bool(goal_handle.is_cancel_requested)

    def sleep_interruptible(self, sec: float, goal_handle, dt: float = 0.05) -> bool:
        t0 = time.time()
        while (time.time() - t0) < float(sec):
            if self.check_abort(goal_handle):
                return False
            time.sleep(float(dt))
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

    def get_axis_value_from_posx(self, posx_list):
        return float(posx_list[PRESS_AXIS_INDEX])

    def compute_pressed_depth(self, contact_pos, current_pos):
        contact_axis = self.get_axis_value_from_posx(contact_pos)
        current_axis = self.get_axis_value_from_posx(current_pos)
        depth = (current_axis - contact_axis) * float(PRESS_AXIS_SIGN)
        return max(0.0, float(depth))

    def get_normal_force_abs(self, ft):
        """
        압착 방향 축 기준 힘 절댓값.
        ft = (fx, fy, fz, tx, ty, tz)
        """
        if ft is None:
            return None

        if PRESS_AXIS_INDEX == 0:
            return abs(float(ft[0]))
        if PRESS_AXIS_INDEX == 1:
            return abs(float(ft[1]))
        return abs(float(ft[2]))

    def publish_feedback(
        self,
        goal_handle,
        state: str,
        ft=None,
        pressed_depth: float = 0.0,
        progress: float = 0.0,
    ):
        fb = PlaceTile.Feedback()
        fb.state = state

        if ft is not None:
            fb.fx = float(ft[0])
            fb.fy = float(ft[1])
            fb.fz = float(ft[2])
            fb.tx = float(ft[3])
            fb.ty = float(ft[4])
            fb.tz = float(ft[5])
        else:
            fb.fx = 0.0
            fb.fy = 0.0
            fb.fz = 0.0
            fb.tx = 0.0
            fb.ty = 0.0
            fb.tz = 0.0

        fb.pressed_depth = float(pressed_depth)
        fb.progress = float(progress)
        goal_handle.publish_feedback(fb)

    # ----------------------------
    # placement preset
    # ----------------------------
    def get_pre_place_pos(self, placement_index: int):
        from DSR_ROBOT2 import posx

        placement_positions = {
            1: [-20.954, 15.683, 104.247, 80.223, 107.752, -32.848],
            2: [-20.954, 15.683, 104.247, 80.223, 107.752, -32.848],
            3: [-20.954, 15.683, 104.247, 80.223, 107.752, -32.848],
            4: [-20.954, 15.683, 104.247, 80.223, 107.752, -32.848],
            5: [-20.954, 15.683, 104.247, 80.223, 107.752, -32.848],
            6: [-20.954, 15.683, 104.247, 80.223, 107.752, -32.848],
            7: [-20.954, 15.683, 104.247, 80.223, 107.752, -32.848],
            8: [-20.954, 15.683, 104.247, 80.223, 107.752, -32.848],
            9: [-20.954, 15.683, 104.247, 80.223, 107.752, -32.848],
        }
        if placement_index not in placement_positions:
            raise ValueError(f"Invalid placement_index: {placement_index}")

        return posx(placement_positions[placement_index])

    # ----------------------------
    # Main press logic
    # ----------------------------
    def press_until_depth_or_force(
        self,
        goal_handle,
        timeout_s: float = 20.0,
        log_dt: float = 0.1,
    ):
        """
        contact 단계 시작 시점 좌표를 기준으로
        현재까지 얼마나 눌렸는지(depth)를 계속 계산/feedback.
        target_press_depth 도달 시 종료.
        """
        from DSR_ROBOT2 import (
            set_ref_coord,
            task_compliance_ctrl,
            set_desired_force,
            check_force_condition,
            get_current_posx,
            wait,
            release_force,
            DR_FC_MOD_REL,
            DR_AXIS_Z,
            DR_BASE,
            DR_TOOL,
        )

        req = goal_handle.request
        max_press_force = float(req.max_press_force)
        target_press_depth = float(req.target_press_depth)

        STX_Z = 20
        STX_ROL = 10
        STX_PIT = 10
        STX_YAW = 10
        SEARCH_FORCE = min(30.0, max_press_force)

        self.get_logger().info(
            f"[PRESS] max_press_force={max_press_force:.3f}, "
            f"target_press_depth={target_press_depth:.3f}, "
            f"PRESS_AXIS_INDEX={PRESS_AXIS_INDEX}, SIGN={PRESS_AXIS_SIGN}"
        )

        touched = False
        contact_start_pos = None   # contact 단계 시작 시점 좌표
        contact_pos = None         # 실제 접촉 감지 시점 좌표
        press_depth = 0.0

        try:
            # -------------------------
            # 0) contact 단계 시작 기준 좌표 저장
            # -------------------------
            contact_start_pos, _ = get_current_posx(DR_BASE)
            self.get_logger().info(f"[CONTACT_START] pos={contact_start_pos}")

            # -------------------------
            # 1) 접촉 탐색
            # -------------------------
            set_ref_coord(DR_TOOL)
            task_compliance_ctrl(
                stx=[3000, 3000, STX_Z, STX_ROL, STX_PIT, STX_YAW],
                time=0.0
            )
            wait(0.2)

            set_desired_force(
                fd=[0, 0, SEARCH_FORCE, 0, 0, 0],
                dir=[0, 0, 1, 0, 0, 0],
                mod=DR_FC_MOD_REL
            )

            t0 = time.time()
            last_log = 0.0

            while (time.time() - t0) < float(timeout_s):
                if self.check_abort(goal_handle):
                    return (False, press_depth, "canceled")

                now = time.time()
                current_pos, _ = get_current_posx(DR_BASE)
                press_depth = self.compute_pressed_depth(contact_start_pos, current_pos)
                ft = self.read_ft_guess()

                if (now - last_log) >= float(log_dt):
                    if ft:
                        self.get_logger().info(
                            f"[FT][CONTACT] "
                            f"Fx={ft[0]:7.2f} Fy={ft[1]:7.2f} Fz={ft[2]:7.2f} | "
                            f"Tx={ft[3]:7.2f} Ty={ft[4]:7.2f} Tz={ft[5]:7.2f} | "
                            f"depth={press_depth:.3f}"
                        )

                    self.publish_feedback(
                        goal_handle,
                        state="contact",
                        ft=ft,
                        pressed_depth=press_depth,
                        progress=min(0.25 + 0.25 * (press_depth / max(target_press_depth, 1e-6)), 0.5),
                    )
                    last_log = now

                # 실제 접촉 판정
                if check_force_condition(DR_AXIS_Z, min=0, max=10.0) == -1:
                    contact_pos, _ = get_current_posx(DR_BASE)
                    touched = True

                    self.get_logger().info(
                        f"[CONTACT] touched=True pos={contact_pos} depth={press_depth:.3f}"
                    )

                    # 접촉 판정 이후에도 같은 기준점(contact_start_pos)으로 계속 depth 누적
                    break

                # 접촉 전에 이미 목표 깊이에 도달하면 종료
                if press_depth >= target_press_depth:
                    self.get_logger().info(
                        f"[PRESS] target depth reached during contact phase: {press_depth:.3f}"
                    )
                    return (True, press_depth, "press_depth_reached")

                wait(0.05)

            if contact_start_pos is None:
                self.get_logger().warn("[CONTACT] failed: no start pos")
                return (False, 0.0, "contact_start_missing")

            # 접촉 못 했더라도 깊이가 목표에 도달했으면 성공 처리 가능
            if not touched:
                current_pos, _ = get_current_posx(DR_BASE)
                press_depth = self.compute_pressed_depth(contact_start_pos, current_pos)

                if press_depth >= target_press_depth:
                    self.get_logger().info(
                        f"[PRESS] target depth reached without explicit contact flag: {press_depth:.3f}"
                    )
                    return (True, press_depth, "press_depth_reached")

                self.get_logger().warn("[CONTACT] failed (timeout)")
                return (False, press_depth, "contact_failed")

            # -------------------------
            # 2) 목표 힘으로 프레싱
            # -------------------------
            release_force()
            wait(0.05)

            set_desired_force(
                fd=[0, 0, max_press_force, 0, 0, 0],
                dir=[0, 0, 1, 0, 0, 0],
                mod=DR_FC_MOD_REL
            )

            press_t0 = time.time()
            last_log = 0.0

            while (time.time() - press_t0) < float(timeout_s):
                if self.check_abort(goal_handle):
                    return (False, press_depth, "canceled")

                current_pos, _ = get_current_posx(DR_BASE)
                press_depth = self.compute_pressed_depth(contact_start_pos, current_pos)
                ft = self.read_ft_guess()

                progress = 0.5 + 0.5 * min(press_depth / max(target_press_depth, 1e-6), 1.0)

                if (time.time() - last_log) >= float(log_dt):
                    if ft:
                        self.get_logger().info(
                            f"[FT][PRESS ] "
                            f"Fx={ft[0]:7.2f} Fy={ft[1]:7.2f} Fz={ft[2]:7.2f} | "
                            f"Tx={ft[3]:7.2f} Ty={ft[4]:7.2f} Tz={ft[5]:7.2f} | "
                            f"depth={press_depth:.3f}"
                        )

                    self.publish_feedback(
                        goal_handle,
                        state="pressing",
                        ft=ft,
                        pressed_depth=press_depth,
                        progress=progress,
                    )
                    last_log = time.time()

                # 목표 깊이 도달 -> 성공 종료
                if press_depth >= target_press_depth:
                    self.get_logger().info(
                        f"[PRESS] target depth reached: {press_depth:.3f}"
                    )
                    return (True, press_depth, "press_depth_reached")

                # 힘 제한 초과 -> 실패 종료
                if ft is not None:
                    force_abs = self.get_normal_force_abs(ft)
                    if force_abs is not None and force_abs >= max_press_force:
                        self.get_logger().warn(
                            f"[PRESS] force limit reached first: "
                            f"{force_abs:.3f} >= {max_press_force:.3f}"
                        )
                        return (False, press_depth, "force_limit_reached")

                wait(0.05)

            self.get_logger().warn("[PRESS] timeout")
            return (False, press_depth, "press_timeout")

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
        from DSR_ROBOT2 import (
            posj,
            movej,
            mwait,
            wait,
            get_current_posx,
            DR_BASE,
            movel,
            posx,
        )

        def move_relative(dx: float, dy: float, dz: float):
            cur, _ = get_current_posx(DR_BASE)
            target = [
                cur[0] + dx,
                cur[1] + dy,
                cur[2] + dz,
                cur[3],
                cur[4],
                cur[5],
            ]
            movel(posx(target), ref=DR_BASE, vel=30, acc=30)
            mwait()

        if self.check_abort(goal_handle):
            return (False, 0.0, "canceled")

        # 1) Home
        self.publish_feedback(goal_handle, "approach", None, 0.0, 0.05)
        j_ready = posj([0, 0, 90, 0, 90, 0])
        movej(j_ready, vel=VELOCITY, acc=ACC)
        mwait()

        if self.check_abort(goal_handle):
            return (False, 0.0, "canceled")

        # 2) placement index별 pre_place
        placement_index = int(goal_handle.request.placement_index)
        pre_place = self.get_pre_place_pos(placement_index)

        self.publish_feedback(goal_handle, "approach", None, 0.0, 0.15)
        movel(pre_place, vel=VELOCITY, acc=ACC)
        mwait()

        if self.check_abort(goal_handle):
            return (False, 0.0, "canceled")

        # 3) 접근 하강
        self.publish_feedback(goal_handle, "approach", None, 0.0, 0.20)
        move_relative(0.0, 0.0, -100.0)

        if self.check_abort(goal_handle):
            return (False, 0.0, "canceled")

        # 4) 압착
        ok, press_depth, message = self.press_until_depth_or_force(
            goal_handle,
            timeout_s=20.0,
            log_dt=0.1,
        )

        if self.check_abort(goal_handle):
            return (False, press_depth, "canceled")

        # 5) 릴리즈/그리퍼 오픈
        self.publish_feedback(
            goal_handle,
            "release",
            self.read_ft_guess(),
            press_depth,
            0.95,
        )
        wait(0.3)
        self.gripper.open_gripper()
        wait(0.5)

        # 6) 복귀
        self.publish_feedback(goal_handle, "release", None, press_depth, 0.98)
        movej(j_ready, vel=VELOCITY, acc=ACC)
        mwait()

        self.publish_feedback(goal_handle, "release", None, press_depth, 1.0)

        if ok:
            return (True, press_depth, message)
        return (False, press_depth, message)


def main(args=None):
    rclpy.init(args=args)

    # ----------------------------
    # boot node 패턴
    # ----------------------------
    boot = rclpy.create_node("dsr_boot_place_tile", namespace=ROBOT_ID)

    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    DR_init.__dsr__node = boot

    # 중요:
    # boot node 설정 후 import 해야 DSR_ROBOT2 내부 g_node가 None이 안 됨
    import DSR_ROBOT2  # noqa: F401

    node = PlaceTileActionServer(boot_node=boot)

    ex = SingleThreadedExecutor()
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