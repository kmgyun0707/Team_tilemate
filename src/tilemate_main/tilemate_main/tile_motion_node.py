#!/usr/bin/env python3
# tilemate_main/tile_motion_node.py

import time
import traceback

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, Float64, String

import DR_init
from tilemate_main.robot_config import RobotConfig


# ============================================================
# 2x2 Pick & Place 데이터/유틸 (원본 스크립트에서 Modbus 제거 버전)
# ============================================================

# ----------------------------
# 타일링 파라미터 (원본 유지)
# ----------------------------
VELOCITY = 30
ACC = 30

OPEN_W = 0.025
CLOSE_W = 0.015

PICK_DY_MM = 32.5

PICK_ABOVE_BASE = [345, 13, 200, 12, -180, 102]
PICK_DOWN_BASE  = [345, 13, 165, 168, -180, -102]
PICK_MOVE_BASE  = [415, 13, 200, 168, -180, -102]

PLACE_TILT_BASE01 = [415, 175, 200, 103, 173, -163]
PLACE_TILT_BASE02 = [415, 105, 200, 103, 173, -163]
PLACE_TILT_BASE03 = [415,  35, 200, 103, 173, -163]
PLACE_TILT_BASE04 = [415, -35, 200, 103, 173, -163]
PLACE_TILT_BASE05 = [415,-105, 200, 103, 173, -163]

PLACE_TILT_BASE06 = [495, 175, 200, 103, 173, -163]
PLACE_TILT_BASE07 = [495, 105, 200, 103, 173, -163]
PLACE_TILT_BASE08 = [495,  35, 200, 103, 173, -163]
PLACE_TILT_BASE09 = [495, -35, 200, 103, 173, -163]
PLACE_TILT_BASE10 = [495,-105, 200, 103, 173, -163]

PLACE_TILT_BASE11 = [575, 175, 200, 103, 173, -163]
PLACE_TILT_BASE12 = [575, 105, 200, 103, 173, -163]
PLACE_TILT_BASE13 = [575,  35, 200, 103, 173, -163]
PLACE_TILT_BASE14 = [575, -35, 200, 103, 173, -163]
PLACE_TILT_BASE15 = [575,-105, 200, 103, 173, -163]

PLACE_TILT_GRID = {
    (0,0): PLACE_TILT_BASE01, (0,1): PLACE_TILT_BASE02, (0,2): PLACE_TILT_BASE03, (0,3): PLACE_TILT_BASE04, (0,4): PLACE_TILT_BASE05,
    (1,0): PLACE_TILT_BASE06, (1,1): PLACE_TILT_BASE07, (1,2): PLACE_TILT_BASE08, (1,3): PLACE_TILT_BASE09, (1,4): PLACE_TILT_BASE10,
    (2,0): PLACE_TILT_BASE11, (2,1): PLACE_TILT_BASE12, (2,2): PLACE_TILT_BASE13, (2,3): PLACE_TILT_BASE14, (2,4): PLACE_TILT_BASE15,
}

PLACE_PLAN_2x2 = [
    (0,0,1),
    (0,1,1),
    (1,0,1),
    (1,1,1),
]

# 기준 RPY 덮어쓰기용
PLACE_TILT_BASE  = [415, 175, 200, 103, 173, -163]
PLACE_DOWN_BASE  = [415, 175, 170,  87, 125,  178]
PLACE_MOVE_BASE1 = [415, 175, 145,  87, 125,  178]
PLACE_MOVE_BASE2 = [415, 250, 140,  87, 125,  178]
PLACE_MOVE_BASE3 = [415, 155, 135,  87, 125,  178]
PLACE_MOVE_BASE4 = [415, 250, 130,  87, 125,  178]
PLACE_MOVE_BASE5 = [415, 150, 140,  87, 125,  178]

REL_DOWN = (0.0,   0.0, -30.0)
REL_M1   = (0.0,   0.0, -55.0)
REL_M2   = (0.0, -10.0, -60.0)
REL_M3   = (0.0, -40.0, -65.0)
REL_M4   = (0.0, -15.0, -65.0)
REL_M5   = (0.0,  -5.0, -65.0)


def add_xy_offset(posx, dx_mm: float, dy_mm: float):
    p = posx[:]
    p[0] += dx_mm
    p[1] += dy_mm
    return p

def add_xyz_offset_keep_rpy(posx, dx_mm: float, dy_mm: float, dz_mm: float):
    p = posx[:]
    p[0] += dx_mm
    p[1] += dy_mm
    p[2] += dz_mm
    return p

def apply_rpy(dst_posx, rpy_src_posx):
    p = dst_posx[:]
    p[3], p[4], p[5] = rpy_src_posx[3], rpy_src_posx[4], rpy_src_posx[5]
    return p

def get_place_tilt_nm(n: int, m: int, offset: int = 1):
    if (n, m) not in PLACE_TILT_GRID:
        raise ValueError(f"Invalid (n,m)=({n},{m})")

    base = PLACE_TILT_GRID[(n, m)][:]

    # offset 규칙: 1 -> 0mm, 2 -> +45mm
    # ⚠️ 너 원본 코드가 n_offset_mm=0.0로 되어있어서 그대로 유지.
    if offset == 1:
        n_offset_mm = 0.0
    elif offset == 2:
        n_offset_mm = 45.0  # ✅ 주석대로면 +45mm가 맞아서 이걸 적용하는게 정상
    else:
        raise ValueError(f"Invalid offset={offset} (only 1 or 2)")

    base[0] += n_offset_mm
    return base


# ============================================================
# Gripper thin client (기존 gripper_node 사용)
# ============================================================
class _GripperClient:
    """tile_motion_node에서 gripper_node로 폭 명령을 보내기 위한 thin client."""
    def __init__(self, node: Node):
        self._node = node
        self._pub = node.create_publisher(Float64, "/gripper/width_m", 10)


    def set_width(self, width_m: float):
        msg = Float64()
        msg.data = float(width_m)
        self._pub.publish(msg)
        self._node.get_logger().info(f"[GRIPPER->CMD] width_m={msg.data:.4f}")


# ============================================================
# TileMotionNode
# ============================================================
class TileMotionNode(Node):
    """
    - /tile/run_once (Int32 token) 수신 시 2x2 타일링 1회 수행
    - /tile/status 로 done/error 발행
    - pause/stop_soft는 기존 토픽 그대로 공유 (/task/pause, /task/stop_soft)
    """
    def __init__(self, cfg: RobotConfig, boot_node: Node):
        super().__init__("tile_motion_node", namespace=cfg.robot_id)
        self.cfg = cfg
        self._boot_node = boot_node

        self._pause = False
        self._stop_soft = False
        self._pending_token = None
        self._running = False

        self.pub_status = self.create_publisher(String, "/tile/status", 10)

        # ✅ completed/jobs (픽앤플레이스 run_once 완료 누적)
        self.pub_completed_jobs = self.create_publisher(Int32, "/completed/jobs", 10)
        self._completed_jobs = 0
        self._publish_completed_jobs()

        self.create_subscription(Int32, "/tile/run_once", self._cb_run_once, 10)
        self.create_subscription(Bool,  "/task/pause", self._cb_pause, 10)
        self.create_subscription(Bool,  "/task/stop_soft", self._cb_stop_soft, 10)


        self.gripper = _GripperClient(self)

        self._initialize_robot()
        self.get_logger().info("TileMotionNode ready: sub /tile/run_once")


    def _initialize_robot(self):
        from DSR_ROBOT2 import set_tool, set_tcp, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS, set_robot_mode
        self.get_logger().info("[TILE] initialize_robot()")
        set_robot_mode(ROBOT_MODE_MANUAL)
        set_tool(self.cfg.tool)
        set_tcp(self.cfg.tcp)
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        time.sleep(1.0)

    def _publish_completed_jobs(self):
        m = Int32()
        m.data = int(self._completed_jobs)
        self.pub_completed_jobs.publish(m)
        self.get_logger().info(f"[TILE] /completed/jobs={m.data}")

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

    def _wait_if_paused(self):
        while rclpy.ok() and self._pause and not self._stop_soft:
            time.sleep(0.05)

    def _publish_status(self, s: str):
        m = String()
        m.data = s
        self.pub_status.publish(m)

    def tick(self):
        """main 루프에서 주기적으로 호출. pending 있으면 동기 실행."""
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
                self.get_logger().warn("[TILE] stop_soft=True during pause -> abort")
                return

            self.get_logger().info(f"[TILE] run_once start token={tok}")
            self._perform_task_2x2()
            # ✅ 성공 완료 시에만 1 증가
            if not self._stop_soft:
                self._completed_jobs += 1
                self._publish_completed_jobs()

            self._publish_status(f"done:{tok}")

        except Exception as e:
            self.get_logger().error(f"[TILE] exception: {e}")
            self.get_logger().error(traceback.format_exc())
            self._publish_status(f"error:{tok}:{e}")

        finally:
            self._running = False

    def _perform_task_2x2(self):
        """원본 perform_task(node) 내용을 그대로 이식 (set_gripper -> self.gripper.set_width)"""
        from DSR_ROBOT2 import movej, movel, wait

        JReady = [0, 0, 90, 0, 90, 90]

        self.get_logger().info("[TILE] Move to JReady")
        movej(JReady, vel=VELOCITY, acc=ACC)

        self.get_logger().info("[TILE] Gripper OPEN init")
        self.gripper.set_width(OPEN_W)
        time.sleep(0.3)

        total = len(PLACE_PLAN_2x2)

        for k, (n, m, offset) in enumerate(PLACE_PLAN_2x2):
            if self._stop_soft:
                self.get_logger().warn("[TILE] stop_soft=True -> break loop")
                break

            self._wait_if_paused()
            if self._stop_soft:
                break

            g = k
            pick_dy = g * PICK_DY_MM

            pick_above = add_xy_offset(PICK_ABOVE_BASE, 0.0, pick_dy)
            pick_down  = add_xy_offset(PICK_DOWN_BASE,  0.0, pick_dy)
            pick_move  = add_xy_offset(PICK_MOVE_BASE,  0.0, pick_dy)

            place_tilt = get_place_tilt_nm(n, m, offset)

            place_down = apply_rpy(add_xyz_offset_keep_rpy(place_tilt, *REL_DOWN), PLACE_DOWN_BASE)
            place_m1   = apply_rpy(add_xyz_offset_keep_rpy(place_tilt, *REL_M1),   PLACE_MOVE_BASE1)
            place_m2   = apply_rpy(add_xyz_offset_keep_rpy(place_tilt, *REL_M2),   PLACE_MOVE_BASE2)
            place_m3   = apply_rpy(add_xyz_offset_keep_rpy(place_tilt, *REL_M3),   PLACE_MOVE_BASE3)
            # place_m4   = apply_rpy(add_xyz_offset_keep_rpy(place_tilt, *REL_M4),   PLACE_MOVE_BASE4)
            # place_m5   = apply_rpy(add_xyz_offset_keep_rpy(place_tilt, *REL_M5),   PLACE_MOVE_BASE5)

            self.get_logger().info(f"[TILE] [{k+1}/{total}] (n,m,offset)=({n},{m},{offset}) pick_dy={pick_dy:.1f}")
            self.get_logger().info(f"[TILE] place_tilt={place_tilt}")
            self.get_logger().info(f"[TILE] place_down={place_down}")

            # ---------------- PICK ----------------
            movel(pick_above, vel=VELOCITY, acc=ACC)
            movel(pick_down,  vel=10, acc=10)

            self.gripper.set_width(CLOSE_W)
            wait(0.3)

            movel(pick_above, vel=VELOCITY, acc=ACC)
            movel(pick_move,  vel=VELOCITY, acc=ACC)

            # ---------------- PLACE ----------------
            movel(place_tilt, vel=VELOCITY, acc=ACC)
            movel(place_down, vel=5, acc=5)

            movel(place_m1, vel=10, acc=10)
            movel(place_m2, vel=10, acc=10)

            self.gripper.set_width(OPEN_W)
            time.sleep(0.3)

            movel(place_m3,  vel=10, acc=10)
            movel(place_tilt, vel=VELOCITY, acc=ACC)

        self.get_logger().info("[TILE] Finish: Move to JReady")
        movej(JReady, vel=VELOCITY, acc=ACC)


# ============================================================
# Main (DSR import 패턴 + spin_once 루프)
# ============================================================
def main(args=None):
    rclpy.init(args=args)
    cfg = RobotConfig()

    # ✅ DSR_ROBOT2 import 전에 boot node 먼저 만들고 DR_init.__dsr__node 세팅
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
