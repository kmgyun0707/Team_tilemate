#!/usr/bin/env python3
# tilemate_main/tile_motion_node.py
# 9 tiles placement ROS2 node (merged)

import time
import traceback

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, Float64, String

import DR_init
from tilemate_main.robot_config import RobotConfig


# ----------------------------
# params
# ----------------------------
VELOCITY = 30
ACC = 30

OPEN_W  = 0.040  # gripper open (m)
CLOSE_W = 0.005  # gripper close (m)


# ----------------------------
# positions (absolute posx list)
# ----------------------------
# pick positions (A/B)
PICK_ABOVE_A = [344.54, -100.1, 260.95, 74.35, 178.8, 73.81]
PICK_ABOVE_B = [436.16, -98.84, 260.78, 69.2, 178.73, 68.63]

# place positions (1..9)
PLACE_TILT_BASE01 = [402.08, 158.83, 229.77, 75.71, 178.59, 76.82]
PLACE_TILT_BASE02 = [469.39, 157.97, 228.80,  69.56, 178.45, 70.97]
PLACE_TILT_BASE03 = [539.79, 156.41, 229.13, 70.08, 178.43, 71.37]

PLACE_TILT_BASE04 = [401.85, 90.05,  229.55, 83.17, 178.79, 84.01]
PLACE_TILT_BASE05 = [469.94, 89.92,  228.78, 75.87, 178.68, 76.88]
PLACE_TILT_BASE06 = [539.63, 88.61,  228.16, 71.45, 178.57, 72.51]

PLACE_TILT_BASE07 = [401.19, 22.88,  228.09, 95.78, 178.83, 96.38]
PLACE_TILT_BASE08 = [468.62, 21.5,   227.98, 85.68, 178.79, 86.43]
PLACE_TILT_BASE09 = [539.05, 20.33,  227.54, 80.24, 178.71, 81.15]

# tool grip (압착판 도구 잡기)
TOOL_GRIP_ABOVE = [531.2, -101.3, 210, 169.29, 177.87, 169.98]
TOOL_GRIP_DOWN  = [531.2, -101.3, 165, 169.29, 177.87, 169.98]
TOOL_WAYPOINT   = [470, 24, 230, 6, -179, 97]


class _GripperClient:
    def __init__(self, node: Node):
        self._node = node
        self._pub = node.create_publisher(Float64, "/gripper/width_m", 10)

    def set_width(self, width_m: float):
        msg = Float64()
        msg.data = float(width_m)
        self._pub.publish(msg)
        self._node.get_logger().info(f"[GRIPPER->CMD] width_m={msg.data:.4f}")

    def grab(self):
        self._node.get_logger().info("[GRIPPER] grab()")
        self.set_width(CLOSE_W)
        time.sleep(1.0)

    def release(self):
        self._node.get_logger().info("[GRIPPER] release()")
        self.set_width(OPEN_W)
        time.sleep(1.0)


class TileMotionNode(Node):
    def __init__(self, cfg: RobotConfig, boot_node: Node):
        super().__init__("tile_motion_node", namespace=cfg.robot_id)
        self.cfg = cfg
        self._boot_node = boot_node

        self._pause = False
        self._stop_soft = False
        self._pending_token = None
        self._running = False

        # add_tcp 1회만 수행하기 위한 플래그
        self._detach_tcp_added = False

        # pubs
        self.pub_status = self.create_publisher(String, "/tile/status", 10)
        self.pub_state = self.create_publisher(String, "/robot/state", 10)
        self.pub_step = self.create_publisher(Int32, "/robot/step", 10)
        self.pub_completed_jobs = self.create_publisher(Int32, "/robot/completed_jobs", 10)
        self._completed_jobs = 0

        # design pattern: default all A
        self._design_pattern = ['A'] * 9

        # subs
        self.create_subscription(Int32, "/tile/run_once", self._cb_run_once, 10)
        self.create_subscription(Bool,  "/task/pause", self._cb_pause, 10)
        self.create_subscription(Bool,  "/task/stop_soft", self._cb_stop_soft, 10)
        self.create_subscription(String, "/robot/design_ab", self._cb_design_ab, 10)

        self.gripper = _GripperClient(self)

        self._initialize_robot()
        self.get_logger().info("TileMotionNode ready: sub /tile/run_once")

    # -----------------
    # init / helpers
    # -----------------
    def _initialize_robot(self):
        from DSR_ROBOT2 import set_tool, set_tcp, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS, set_robot_mode
        self.get_logger().info("[TILE] initialize_robot()")
        set_robot_mode(ROBOT_MODE_MANUAL)
        set_tool(self.cfg.tool)
        set_tcp(self.cfg.tcp)
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        time.sleep(1.0)

    def _set_robot_status(self, step: int, state: str):
        m_step = Int32()
        m_step.data = int(step)
        m_state = String()
        m_state.data = str(state)
        self.pub_step.publish(m_step)
        self.pub_state.publish(m_state)
        self.get_logger().info(f"[STATUS] step={m_step.data} state='{m_state.data}'")

    def _publish_status(self, s: str):
        m = String()
        m.data = s
        self.pub_status.publish(m)
        self.get_logger().info(f"[TILE->STATUS] {m.data}")

    def _wait_if_paused(self):
        while rclpy.ok() and self._pause and not self._stop_soft:
            time.sleep(0.05)

    def _increase_completed_jobs(self):
        """타일 1장 완료 카운트 증가 + publish"""
        self._completed_jobs += 1
        m = Int32()
        m.data = int(self._completed_jobs)
        self.pub_completed_jobs.publish(m)
        self.get_logger().info(f"[JOBS] completed_jobs: {m.data} / 9")

    # -----------------
    # callbacks
    # -----------------
    def _cb_design_ab(self, msg: String):
        raw = msg.data.strip()
        if not raw:
            return
        self._design_pattern = [x.strip().upper() for x in raw.split(",") if x.strip()]
        self.get_logger().info(f"[TILE] design_ab updated: {self._design_pattern}")

    def _cb_run_once(self, msg: Int32):
        if self._running:
            self.get_logger().warn("[TILE] run_once ignored (already running)")
            return
        self._pending_token = int(msg.data)
        self.get_logger().info(f"[TILE] received token={self._pending_token}")

    def _cb_pause(self, msg: Bool):
        self._pause = bool(msg.data)
        self.get_logger().warn(f"[TILE] pause={self._pause}")

    def _cb_stop_soft(self, msg: Bool):
        self._stop_soft = bool(msg.data)
        self.get_logger().warn(f"[TILE] stop_soft={self._stop_soft}")

    # -----------------
    # main tick
    # -----------------
    def tick(self):
        if self._pending_token is None or self._running:
            return

        tok = self._pending_token
        self._pending_token = None

        if self._stop_soft:
            self.get_logger().warn("[TILE] stop_soft=True -> skip token")
            return

        self._running = True
        try:
            self._wait_if_paused()
            if self._stop_soft:
                self._publish_status(f"error:{tok}:aborted(stop_soft)")
                return

            self.get_logger().info(f"[TILE] run_once start token={tok}")

            ok = self._perform_task_9tiles_design_pattern()

            if ok and not self._stop_soft:
                self._set_robot_status(5, "타일 작업 완료")
                self._publish_status(f"done:{tok}")
            else:
                self._publish_status(f"error:{tok}:aborted/failed")

        except Exception as e:
            self.get_logger().error(f"[TILE] exception: {e}")
            self.get_logger().error(traceback.format_exc())
            self._publish_status(f"error:{tok}:{e}")

        finally:
            self._running = False

    # ============================================================
    # Tool grip before start: release -> go above/down -> grab -> above -> waypoint
    # ============================================================
    def _grip_tool_before_start(self) -> bool:
        from DSR_ROBOT2 import movel, posx, wait, DR_BASE

        self._wait_if_paused()
        if self._stop_soft:
            return False

        self.get_logger().info("[TOOL] grip tool start")

        # open gripper
        # self._set_robot_status(10, "툴 집기: 그리퍼 열기")
        self.gripper.release()

        # go above/down
        # self._set_robot_status(11, "툴 집기: 접근(상부)")
        movel(posx(TOOL_GRIP_ABOVE), ref=DR_BASE, vel=VELOCITY, acc=ACC)
        wait(0.2)

        # self._set_robot_status(12, "툴 집기: 하강(파지)")
        movel(posx(TOOL_GRIP_DOWN), ref=DR_BASE, vel=VELOCITY, acc=ACC)
        wait(0.2)

        # grab
        # self._set_robot_status(13, "툴 집기: 그리퍼 닫기(파지)")
        self.gripper.grab()
        wait(0.2)

        # up
        # self._set_robot_status(14, "툴 집기: 상승(상부)")
        movel(posx(TOOL_GRIP_ABOVE), ref=DR_BASE, vel=VELOCITY, acc=ACC)
        wait(0.2)

        # waypoint
        # self._set_robot_status(15, "툴 집기: 안전구역(Waypoint) 이동")
        movel(posx(TOOL_WAYPOINT), ref=DR_BASE, vel=VELOCITY, acc=ACC)
        wait(0.2)

        self.get_logger().info("[TOOL] grip tool done")
        return True

    # ============================================================
    # Detach tile: add_tcp once + tile_idx based tilt sign + restore TCP
    # ============================================================
    def detach_tile(self, tile_idx: int, tilt_abs_deg: float = 24.0) -> bool:
        """
        tile_idx:
          - 3,6,9 => tilt = -tilt_abs_deg
          - else  => tilt = +tilt_abs_deg
        add_tcp는 1회만, 끝나면 이전 TCP로 복귀(restore)
        """
        from DSR_ROBOT2 import (
            posx, movel, wait,
            DR_TOOL,
            add_tcp, get_tcp, set_tcp,
            set_robot_mode, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS
        )

        self._wait_if_paused()
        if self._stop_soft:
            return False

        tilt_angle = -float(tilt_abs_deg) if (tile_idx % 3 == 0) else float(tilt_abs_deg)

        # save current TCP
        try:
            prev_tcp = get_tcp()
        except Exception as e:
            self.get_logger().warn(f"[DETACH] get_tcp() failed: {e}")
            prev_tcp = None

        tcp_name = "MySuction_v1"
        tcp_offset = [0, 0, 265, 0, 0, 0]

        try:
            set_robot_mode(ROBOT_MODE_MANUAL)

            if not self._detach_tcp_added:
                try:
                    add_tcp(tcp_name, tcp_offset)
                    self.get_logger().info(f"[DETACH] add_tcp done: {tcp_name} offset={tcp_offset}")
                except Exception as e:
                    self.get_logger().warn(f"[DETACH] add_tcp failed (maybe exists): {e}")
                finally:
                    self._detach_tcp_added = True

            set_tcp(tcp_name)
            set_robot_mode(ROBOT_MODE_AUTONOMOUS)
            wait(0.2)

            self.get_logger().info(f"[DETACH] tile_idx={tile_idx} tilt={tilt_angle}deg")
            tilt_forward = posx([0, 0, 0, 0, tilt_angle, 0])
            movel(tilt_forward, vel=1, acc=1, ref=DR_TOOL, time=0.5)
            wait(0.2)

            return True

        except Exception as e:
            self.get_logger().error(f"[DETACH] exception: {e}")
            self.get_logger().error(traceback.format_exc())
            return False

        finally:
            # restore TCP
            try:
                if prev_tcp is not None:
                    set_robot_mode(ROBOT_MODE_MANUAL)
                    set_tcp(prev_tcp)
                    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
                    wait(0.2)
                    self.get_logger().info(f"[DETACH] TCP restored: {prev_tcp}")
            except Exception as e:
                self.get_logger().warn(f"[DETACH] restore failed: {e}")

    # ============================================================
    # Compliant approach (force-based descent)
    # ============================================================
    def compliant_approach(self, threshold_n: float = 5.0, timeout_s: float = 10.0) -> bool:
        from DSR_ROBOT2 import (
            set_ref_coord, task_compliance_ctrl, set_desired_force,
            check_force_condition, release_force, release_compliance_ctrl,
            DR_TOOL, DR_FC_MOD_REL, DR_AXIS_Z, DR_BASE, wait
        )

        self._wait_if_paused()
        if self._stop_soft:
            return False

        self.get_logger().info(f"[COMPLIANT] start (threshold={threshold_n}N, timeout={timeout_s}s)")

        set_ref_coord(DR_TOOL)
        task_compliance_ctrl(stx=[3000, 3000, 50, 200, 200, 200])
        wait(0.3)

        set_desired_force(
            fd=[0, 0, float(threshold_n + 15), 0, 0, 0],
            dir=[0, 0, 1, 0, 0, 0],
            mod=DR_FC_MOD_REL
        )

        t0 = time.time()
        try:
            while True:
                self._wait_if_paused()
                if self._stop_soft:
                    return False

                if timeout_s is not None and (time.time() - t0) > float(timeout_s):
                    self.get_logger().warn("[COMPLIANT] timeout -> fail")
                    return False

                ret = check_force_condition(DR_AXIS_Z, min=0, max=float(threshold_n))
                if ret == -1:
                    self.get_logger().info(f"[COMPLIANT] reached threshold={threshold_n}N")
                    return True

                wait(0.1)

        finally:
            try:
                release_force()
            except Exception:
                pass
            try:
                release_compliance_ctrl()
            except Exception:
                pass
            try:
                set_ref_coord(DR_BASE)
            except Exception:
                pass
            wait(0.2)

    # ============================================================
    # Main: 9 tiles with design pattern (A/B), per-tile completed_jobs +1
    # ============================================================
    def _perform_task_9tiles_design_pattern(self) -> bool:
        from DSR_ROBOT2 import movej, movel, posx, wait, DR_BASE

        # tool grip first
        # self._set_robot_status(9, "작업 시작 전: 툴 집기")
        if not self._grip_tool_before_start():
            self.get_logger().warn("[TILE] tool grip failed/aborted")
            return False

        # JReady
        JReady = [0, 0, 90, 0, 90, 90]
        self._set_robot_status(0, "JReady 이동")
        movej(JReady, vel=VELOCITY, acc=ACC)
        wait(0.2)

        place_targets = [
            (1, PLACE_TILT_BASE01),
            (2, PLACE_TILT_BASE02),
            (3, PLACE_TILT_BASE03),
            (4, PLACE_TILT_BASE04),
            (5, PLACE_TILT_BASE05),
            (6, PLACE_TILT_BASE06),
            (7, PLACE_TILT_BASE07),
            (8, PLACE_TILT_BASE08),
            (9, PLACE_TILT_BASE09),
        ]

        for tile_idx, place_pos in place_targets:
            self._wait_if_paused()
            if self._stop_soft:
                self.get_logger().warn("[TILE] stop_soft -> abort sequence")
                return False

            # design pattern select (A/B)
            list_index = tile_idx - 1
            tile_type = self._design_pattern[list_index] if (list_index < len(self._design_pattern)) else 'A'

            if tile_type == 'B':
                current_pick_pos = PICK_ABOVE_B
                color_name = "흰색(B)"
            else:
                current_pick_pos = PICK_ABOVE_A
                color_name = "검정(A)"

            # ---------------- PICK ----------------
            self._set_robot_status(1, f"타일 파지 준비 ({color_name}) - {tile_idx}번")
            movel(posx(current_pick_pos), ref=DR_BASE, vel=VELOCITY, acc=ACC)
            wait(0.1)

            self._set_robot_status(2, f"타일 파지 하강(순응) - {tile_idx}번")
            if not self.compliant_approach(threshold_n=5.0, timeout_s=5.0):
                self.get_logger().warn(f"[TILE] pick compliant failed tile_idx={tile_idx}")
                return False

            wait(0.2)
            self._set_robot_status(3, f"타일 파지 상승 - {tile_idx}번")
            movel(posx(current_pick_pos), ref=DR_BASE, vel=VELOCITY, acc=ACC)
            wait(0.1)

            # ---------------- PLACE ----------------
            self._set_robot_status(4, f"타일 배치 상부 이동 - {tile_idx}번")
            movel(posx(place_pos), ref=DR_BASE, vel=VELOCITY, acc=ACC)
            wait(0.1)

            self._set_robot_status(4, f"타일 배치 하강(순응) - {tile_idx}번")
            if not self.compliant_approach(threshold_n=5.0, timeout_s=10.0):
                self.get_logger().warn(f"[TILE] place compliant failed tile_idx={tile_idx}")
                return False

            # ---------------- DETACH ----------------
            ok_detach = self.detach_tile(tile_idx, tilt_abs_deg=24.0)
            self.get_logger().info(f"[TILE] detach result tile_idx={tile_idx} ok={ok_detach}")

            self._set_robot_status(4, f"타일 배치 상부 복귀 - {tile_idx}번")
            movel(posx(place_pos), ref=DR_BASE, vel=VELOCITY, acc=ACC)
            wait(0.1)

            # ✅ per-tile completed_jobs +1
            self._increase_completed_jobs()
            self.get_logger().info(f"🎉 {tile_idx}번 타일 작업 완료!")

        self._set_robot_status(3, "시퀀스 종료(9장 완료)")
        return True


def main(args=None):
    rclpy.init(args=args)
    cfg = RobotConfig()

    boot = rclpy.create_node("dsr_boot_tile", namespace=cfg.robot_id)
    DR_init.__dsr__id = cfg.robot_id
    DR_init.__dsr__model = cfg.robot_model
    DR_init.__dsr__node = boot

    import DSR_ROBOT2  # noqa: F401

    node = TileMotionNode(cfg, boot)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.tick()
    finally:
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