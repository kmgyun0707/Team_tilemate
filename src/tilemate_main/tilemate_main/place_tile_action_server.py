#!/usr/bin/env python3
import time
import rclpy
import DR_init

from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from tilemate_msgs.action import PlaceTile
from tilemate_main.robot_config import RobotConfig, GripperConfig


class PlaceTileActionServer(Node):
    def __init__(self, robot_cfg: RobotConfig, gripper_cfg: GripperConfig, boot_node: Node):
        super().__init__("place_tile_action_server", namespace=robot_cfg.robot_id)

        self._boot_node = boot_node
        self.robot_cfg = robot_cfg
        self.gripper_cfg = gripper_cfg

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

        from tilemate_main.onrobot import RG
        self.gripper = RG(
            gripper_cfg.GRIPPER_NAME,
            gripper_cfg.TOOLCHARGER_IP,
            gripper_cfg.TOOLCHARGER_PORT,
        )
        self.placement_index = 0
        self.initialize_robot()
        self.get_logger().info("\033[94m [4/5] [PLACE_TILE] initialize Done!\033[0m")

    # ----------------------------
    # Action callbacks
    # ----------------------------
    def goal_callback(self, goal_request):
        if goal_request.placement_index < 1:
            self.get_logger().warn("Reject goal: placement_index must be >= 1")
            return GoalResponse.REJECT

        if goal_request.placement_index > 9:
            self.get_logger().warn("Reject goal: placement_index must be <= 9")
            return GoalResponse.REJECT

        if goal_request.max_press_force <= 0.0:
            self.get_logger().warn("Reject goal: max_press_force <= 0")
            return GoalResponse.REJECT

        if goal_request.target_press_depth < 0.0:
            self.get_logger().warn("Reject goal: target_press_depth < 0")
            return GoalResponse.REJECT

        self.placement_index = goal_request.placement_index
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
            try:
                goal_handle.abort()
            except Exception:
                pass
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
            ROBOT_MODE_MANUAL,
            ROBOT_MODE_AUTONOMOUS,
            set_robot_mode,
            wait,
        )

        self.get_logger().info("[PLACE_TILE] initialize_robot()")

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
    # compaction
    # ----------------------------
    def compact_tile(
        self,
        goal_handle,
        timeout_s: float = 30.0,
        log_dt: float = 0.2,
    ):
        from DSR_ROBOT2 import (
            set_ref_coord,
            task_compliance_ctrl,
            set_desired_force,
            check_force_condition,
            get_current_posx,
            get_current_posj,
            amovej,
            wait,
            release_force,
            DR_FC_MOD_REL,
            DR_AXIS_Y,
            DR_BASE,
            DR_TOOL,
        )

        req = goal_handle.request

        STX_Y = 20
        STX_ROL = 10
        STX_PIT = 10
        STX_YAW = 10

        Force = float(req.max_press_force)

        self.get_logger().info(
            f"[SMART_TWIST] start | "
            f"Force={Force:.3f}, Stx_z={STX_Y}, "
            f"Stx_roll={STX_ROL}, Stx_pitch={STX_PIT}, Stx_yaw={STX_YAW}"
        )

        if self.check_abort(goal_handle):
            return (False, 0.0, "canceled")

        set_ref_coord(DR_BASE)
        task_compliance_ctrl(
            stx=[3000, STX_Y, 3000, STX_ROL, STX_PIT, STX_YAW],
            time=0.0
        )
        wait(0.2)

        search_force = min(30.0, Force)
        set_desired_force(
            fd=[0, search_force, 0, 0, 0, 0],
            dir=[0, 1, 0, 0, 0, 0],
            mod=DR_FC_MOD_REL
        )

        t0 = time.time()
        last_log = 0.0
        touched = False
        contact_y = 0.0
        contact_joint = None

        try:
            self.publish_feedback(goal_handle, "contact_search", None, 0.0, 0.30)

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
                    self.publish_feedback(goal_handle, "contact_search", ft, 0.0, 0.40)
                    last_log = now

                if check_force_condition(DR_AXIS_Y, min=0, max=15.0, ref=DR_BASE) == -1:
                    base_pos, _ = get_current_posx(DR_BASE)
                    self.get_logger().info(f"[CONTACT] before_press_pos={base_pos}")

                    touched = True
                    contact_pos, _ = get_current_posx(DR_BASE)
                    contact_y = float(contact_pos[1])
                    contact_joint = get_current_posj()

                    self.get_logger().info(
                        f"[CONTACT] touched=True contact_y={contact_y:.3f}"
                    )

                    release_force()
                    break

                wait(0.1)

            if not touched or contact_joint is None:
                self.get_logger().warn("[CONTACT] failed (timeout/no joint)")
                return (False, 0.0, "contact_failed")

            # amovej(contact_joint, vel=40, acc=40)

            if not self.sleep_interruptible(1.0, goal_handle):
                return (False, 0.0, "canceled")

            final_pos, _ = get_current_posx(DR_BASE)
            final_z = float(final_pos[2])
            depth = float(final_z-contact_y)

            self.get_logger().info(f"[RESULT] final_pose={final_pos} depth={depth:.3f}")
            self.publish_feedback(goal_handle, "pressed", self.read_ft_guess(), depth, 0.90)

            target_depth = float(req.target_press_depth)
            if target_depth > 0.0 and depth < target_depth:
                return (True, depth, "target_depth_not_reached")

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
        from DSR_ROBOT2 import (
            posj,
            movej,
            mwait,
            get_current_posx,
            DR_BASE,
            movel,
            posx,
        )

        def move_relative(dx: float, dy: float, dz: float, dw: float = 0.0, dp: float = 0.0, dr: float = 0.0):
            from DSR_ROBOT2 import posx, movel, mwait, DR_BASE, get_current_posx

            cur, _ = get_current_posx(DR_BASE)
            target = [
                cur[0] + dx,
                cur[1] + dy,
                cur[2] + dz,
                cur[3] + dw,
                cur[4] + dp,
                cur[5] + dr,
            ]
            movel(posx(target), ref=DR_BASE, vel=30, acc=30)
            mwait()

        def move_absoulte(x: float, y: float, z: float, w: float = 0.0, p: float = 0.0, r: float = 0.0):
            from DSR_ROBOT2 import posx, movel, mwait, DR_BASE, get_current_posx

            cur, _ = get_current_posx(DR_BASE)
            target = [
                x,
                y,
                z,
                cur[3] + w,
                cur[4] + p,
                cur[5] + r,
            ]
            movel(posx(target), ref=DR_BASE, vel=30, acc=30)
            mwait()
            print(get_current_posx(DR_BASE))

        def move_to_tile_place_position(placement_index: int):
            from DSR_ROBOT2 import posx, movel, mwait, DR_BASE

            tool_pre_tilt = [369.025, 160.678, 196.749, 44.152, -179.9, -137.748]

            tile_wid = 80.0  # mm

            tile_offsets = {
                1: (-tile_wid,  tile_wid),
                2: (0.0,        tile_wid),
                3: (tile_wid,   tile_wid),
                4: (-tile_wid,  0.0),
                5: (0.0,        0.0),
                6: (tile_wid,   0.0),
                7: (-tile_wid, -tile_wid),
                8: (0.0,       -tile_wid),
                9: (tile_wid,  -tile_wid),
            }

            if placement_index not in tile_offsets:
                raise ValueError(f"invalid placement_index: {placement_index}")

            dx, dz = tile_offsets[placement_index]

            target = tool_pre_tilt.copy()
            target[0] += dx
            target[1] -= 30.0
            target[2] += dz

            print(f"[PLACE_TILE] Index={placement_index} 이동 좌표: {target}")

            movel(posx(target), vel=30, acc=30, ref=DR_BASE)
            mwait()
            return dx, dz

        j_ready = posj([0, 0, 90, 0, 90, 0])


        # 1) Home
        self.publish_feedback(goal_handle, "approach_home", None, 0.0, 0.05)
        movej(j_ready, vel=self.robot_cfg.vel, acc=self.robot_cfg.acc)
        mwait()

        if self.check_abort(goal_handle):
            return (False, press_depth, "canceled")

        move_relative(0.0, 0.0, 0.0, dw=180.0)

        # 2) pre_place 이동
        self.publish_feedback(goal_handle, "approach_pre_place", None, 0.0, 0.15)
        dx, dz = move_to_tile_place_position(self.placement_index)

        if self.check_abort(goal_handle):
            return (False, press_depth, "canceled")

        self.publish_feedback(goal_handle, "approach_target", None, 0.0, 0.25)

        tool_tilt = [369.025, 160.678, 196.749, 91.799, -162.285, -90.1]
        tool_tilt_target = tool_tilt.copy()
        tool_tilt_target[0] += dx
        tool_tilt_target[2] += dz
        movel(posx(tool_tilt_target), 30, 30, ref=DR_BASE)
        mwait()

        if self.check_abort(goal_handle):
            return (False, press_depth, "canceled")

        # 4) 압착
        self.get_logger().info(f"[PLACE_TILE] compact start (tile={self.placement_index})")

        ok, press_depth, message = self.compact_tile(
            goal_handle,
            timeout_s=20.0,
            log_dt=0.1,
        )
        
        if not ok:
            return (False, press_depth, message)


        move_relative(0.0, -23.68, 0.0)
        mwait()

        if self.check_abort(goal_handle):
            return (False, press_depth, "canceled")

        # 중간 홈 복귀
        movej(j_ready, vel=self.robot_cfg.vel, acc=self.robot_cfg.acc)
        mwait()

        if self.check_abort(goal_handle):
            return (False, press_depth, "canceled")

        # 8) 툴 반납 위치 이동
        tool_pre_release = [233.122, -350.591, 143.064]
        tool_release = [233.122, -350.591, 123.0]

        move_absoulte(tool_pre_release[0], tool_pre_release[1], tool_pre_release[2])
        move_absoulte(tool_release[0], tool_release[1], tool_release[2])

        self.gripper.open_gripper()
        
        # 8) 툴 반납후 상단 이동
        move_relative(0.0, 0.0, 30.0)


        # 9) 홈 복귀
        self.publish_feedback(goal_handle, "return_home", None, press_depth, 0.98)
        movej(j_ready, vel=self.robot_cfg.vel, acc=self.robot_cfg.acc)
        mwait()

        self.publish_feedback(goal_handle, "done", None, press_depth, 1.0)

        return (True, press_depth, message)


def main(args=None):
    rclpy.init(args=args)

    robot_cfg = RobotConfig()
    gripper_cfg = GripperConfig()

    boot = rclpy.create_node("dsr_boot_place_tile", namespace=robot_cfg.robot_id)

    DR_init.__dsr__id = robot_cfg.robot_id
    DR_init.__dsr__model = robot_cfg.robot_model
    DR_init.__dsr__node = boot

    import DSR_ROBOT2  # noqa: F401

    node = PlaceTileActionServer(robot_cfg, gripper_cfg, boot)

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