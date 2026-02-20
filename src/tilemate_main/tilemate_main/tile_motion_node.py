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

OPEN_W  = 0.025
CLOSE_W = 0.005

# ----------------------------
# PICK 설정 (공급 라인)
# ----------------------------
# 지금은 Y만 누적해서 공급 위치를 아래로 내려가며 집는 시나리오
PICK_DY_MM = 32.5   # g=0..3 => 0, 32.5, 65, 97.5

PICK_ABOVE_BASE = [345, 13, 200, 12, -180, 102]
PICK_DOWN_BASE  = [345, 13, 165, 168, -180, -102]
PICK_MOVE_BASE  = [415, 13, 200, 168, -180, -102]

# ----------------------------
# PLACE (n,m) TILT 그리드 (너가 추가한 좌표)
# - 이 좌표는 "각 (n,m) 위치로 접근할 때"의 자세를 이미 맞춘 값으로 사용
# ----------------------------
# PLACE_TILT_BASE01 = [415, 175, 200, 103, 173, -163] # 라인 시작 위치
# PLACE_TILT_BASE02 = [415, 105, 200, 103, 173, -163]
# PLACE_TILT_BASE03 = [415,  45, 200, 103, 173, -163]
# PLACE_TILT_BASE04 = [415, -25, 200, 103, 173, -163]
# PLACE_TILT_BASE05 = [415, -95, 200, 103, 173, -163]

# PLACE_TILT_BASE06 = [495,-165, 200, 103, 173, -163]
# PLACE_TILT_BASE07 = [495, 175, 200, 103, 173, -163]
# PLACE_TILT_BASE08 = [495, 115, 200, 103, 173, -163]
# PLACE_TILT_BASE09 = [495,  45, 200, 103, 173, -163]
# PLACE_TILT_BASE10 = [495, -25, 200, 103, 173, -163]

# PLACE_TILT_BASE11 = [575, 175, 200, 103, 173, -163]
# PLACE_TILT_BASE12 = [575, 115, 200, 103, 173, -163]
# PLACE_TILT_BASE13 = [575,  45, 200, 103, 173, -163]
# PLACE_TILT_BASE14 = [575, -25, 200, 103, 173, -163]
# PLACE_TILT_BASE15 = [575, -95, 200, 103, 173, -163]

# pitch 가정: 70mm (타일 60 + 여유 10)

# X = 415 (5개)
PLACE_TILT_BASE01 = [415, 175, 200, 103, 173, -163]
PLACE_TILT_BASE02 = [415, 100, 200, 103, 173, -163]
PLACE_TILT_BASE03 = [415,  40, 200, 103, 173, -163]
PLACE_TILT_BASE04 = [415, -30, 200, 103, 173, -163]
PLACE_TILT_BASE05 = [415,-100, 200, 103, 173, -163]

# X = 495 (5개)  <- BASE06을 여기 첫번째로 재배치
PLACE_TILT_BASE06 = [485, 175, 200, 103, 173, -163]
PLACE_TILT_BASE07 = [485, 100, 200, 103, 173, -163]
PLACE_TILT_BASE08 = [485,  40, 200, 103, 173, -163]
PLACE_TILT_BASE09 = [485, -30, 200, 103, 173, -163]
PLACE_TILT_BASE10 = [485,-100, 200, 103, 173, -163]

# X = 575 (5개)
PLACE_TILT_BASE11 = [575, 175, 200, 103, 173, -163]
PLACE_TILT_BASE12 = [575, 100, 200, 103, 173, -163]
PLACE_TILT_BASE13 = [575,  40, 200, 103, 173, -163]
PLACE_TILT_BASE14 = [575, -30, 200, 103, 173, -163]
PLACE_TILT_BASE15 = [575,-100, 200, 103, 173, -163]


PLACE_TILT_GRID = {
    (0,0): PLACE_TILT_BASE01, (0,1): PLACE_TILT_BASE02, (0,2): PLACE_TILT_BASE03, (0,3): PLACE_TILT_BASE04, (0,4): PLACE_TILT_BASE05,
    (1,0): PLACE_TILT_BASE06, (1,1): PLACE_TILT_BASE07, (1,2): PLACE_TILT_BASE08, (1,3): PLACE_TILT_BASE09, (1,4): PLACE_TILT_BASE10,
    (2,0): PLACE_TILT_BASE11, (2,1): PLACE_TILT_BASE12, (2,2): PLACE_TILT_BASE13, (2,3): PLACE_TILT_BASE14, (2,4): PLACE_TILT_BASE15,
}

# ----------------------------
# 2x2 배치 계획 (n,m,offset)
# - offset: 1 => x +0mm, 2 => x +45mm
# ----------------------------
# 2x2 그리드에서 (n,m) = (0,0), (0,1), (1,0), (1,1) 위치에 타일을 놓는 시나리오
PLACE_PLAN_2x2 = [
    (0,0,1),  # 1) 좌상
    (0,1,1),  # 2) 좌하
    (1,0,1),  # 3) 우상
    (1,1,1),  # 4) 우하
]

# ----------------------------
# "기준 좌표" (한 위치에서 측정한 down/move 단계의 기준)
# - 여기서 RPY를 추출해서 각 단계별로 강제 적용한다.
# - XYZ는 place_tilt 기반으로 상대 오프셋으로 만들고, RPY는 아래 기준의 RPY로 덮어쓴다.
# ----------------------------
PLACE_TILT_BASE  = [415, 175, 200, 103, 173, -163] # (n,m,offset)에서 타일 배치 접근 자세로 이동할 때의 기준 좌표 (위 그리드에서 하나 선택)
PLACE_DOWN_BASE  = [415, 175, 170,  87, 125,  178] # (tilt에서 z만 30mm 내려간 위치, RPY는 down 기준으로 적용)
PLACE_MOVE_BASE1 = [415, 175, 145,  87, 125,  178] # (꺽어 놓기용, 상황에 맞게 조절 필요)
PLACE_MOVE_BASE2 = [415, 250, 140,  87, 125,  178] # (Y를 좀 빼서 슬라이드할 때 뒤로 빠지도록, 상황에 맞게 조절 필요)
PLACE_MOVE_BASE3 = [415, 250, 130,  87, 125,  178] # (조금더 하강용, 상황에 맞게 조절 필요)
PLACE_MOVE_BASE4 = [415, 250, 130,  87, 125,  178] # 후퇴할 때는 Y를 크게 빼서 뒤로 빠지도록 (250은 임의값, 상황에 맞게 조절 필요)
PLACE_MOVE_BASE5 = [415, 150, 140,  87, 125,  178] # (조금더 후 후퇴용, 상황에 맞게 조절 필요)

# ----------------------------
# PLACE 상대 오프셋(tilt 기준)  ※ "XYZ(mm)"만 사용해야 함
# - (중요) 87,125,178 같은 값은 RPY이므로 오프셋으로 쓰면 좌표가 폭발함
# - 아래 오프셋은 위 BASE들에서 XYZ 차이로 계산한 값
# ----------------------------
REL_DOWN = (0.0,   0.0, -30.0)   # 200 -> 170
REL_M1   = (0.0,   0.0, -55.0)   # 200 -> 145
REL_M2   = (0.0, -10.0, -60.0)   # 175 -> 165, 200 -> 140
REL_M3   = (0.0, -10.0, -65.0)   # 175 -> 155, 200 -> 135
REL_M4   = (0.0, -15.0, -65.0)   # 175 -> 160, 200 -> 135
REL_M5   = (0.0,  -5.0, -65.0)   # 175 -> 170, 200 -> 135


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

    ##
    # offset 규칙: 1 -> 0mm, 2 -> +45mm
    # ⚠️ 너 원본 코드가 n_offset_mm=0.0로 되어있어서 그대로 유지.
    if offset == 1:
        n_offset_mm = 0.0
    elif offset == 2:
        n_offset_mm = 0.0  
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
        self.pub_state = self.create_publisher(String, "/robot/state", 10)
        self.pub_step = self.create_publisher(Int32, "/robot/step", 10 )
        self.pub_completed_jobs = self.create_publisher(Int32, "/completed/jobs", 10)
        self._completed_jobs = 0
        #
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

    def _set_robot_status(self, step: int, state: str):
        m_step = Int32()
        m_step.data = step
        m_state = String()
        m_state.data = state
        self.pub_step.publish(m_step)
        self.pub_state.publish(m_state)
        self.get_logger().info(f"[STATUS] step={step} state='{state}'")

    def _publish_completed_jobs(self):
        m = Int32()
        m.data = self._completed_jobs
        self.pub_completed_jobs.publish(m)
        self.get_logger().info(f"[TILE] /completed/jobs={m.data}")

    def _publish_init_state(self):
        m_state = String()
        m_state.data = "타일배치중"
        m_step = Int32()
        m_step.data = 4
        self.pub_state.publish(m_state)
        self.pub_step.publish(m_step)

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
                self._set_robot_status(5, "타일 작업 완료")

            self._publish_status(f"done:{tok}")

        except Exception as e:
            self.get_logger().error(f"[TILE] exception: {e}")
            self.get_logger().error(traceback.format_exc())
            self._publish_status(f"error:{tok}:{e}")

        finally:
            self._running = False

    def _perform_task_2x2(self):
        from DSR_ROBOT2 import movej, movel, wait
        from DSR_ROBOT2 import (
            set_ref_coord, task_compliance_ctrl, release_compliance_ctrl,
            set_desired_force, release_force, check_force_condition,
            DR_BASE, DR_TOOL, DR_AXIS_Z, DR_FC_MOD_REL
        )

        # 힘제어/순응제어 (force control / compliance control)
        def hold_with_force_4n(hold_n=4.0, timeout_s=3.0):
            # 타일 임시 고정: 슬라이드 정렬 끝난 뒤, 오픈 전 Z방향으로 hold_n 만큼 눌러 고정
            set_ref_coord(DR_TOOL)  # Tool 좌표계 기준 (안전)
            task_compliance_ctrl(stx=[1000, 1000, 200, 200, 200, 200])
            wait(0.3)

            set_desired_force(
                fd=[0, 0, hold_n, 0, 0, 0],
                dir=[0, 0, 1, 0, 0, 0],
                mod=DR_FC_MOD_REL
            )

            t0 = time.time()
            while True:
                # NOTE: check_force_condition의 리턴 의미는 환경마다 다를 수 있어 로그로 확인 추천
                ret = check_force_condition(DR_AXIS_Z, min=0, max=hold_n)
                if ret == -1:
                    break

                if time.time() - t0 > timeout_s:
                    self.get_logger().warn("[HOLD] force condition timeout -> release")
                    break

                wait(0.2)

            release_force()
            release_compliance_ctrl()
            set_ref_coord(DR_BASE)
            wait(0.2)

        JReady = [0, 0, 90, 0, 90, 90]

        self.get_logger().info("[TILE] Move to JReady")
        movej(JReady, vel=VELOCITY, acc=ACC)

        self.get_logger().info("[TILE] Gripper OPEN init")
        self.gripper.set_width(OPEN_W)
        time.sleep(0.3)

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
            place_m4   = apply_rpy(add_xyz_offset_keep_rpy(place_tilt, *REL_M4),   PLACE_MOVE_BASE4)
            place_m5   = apply_rpy(add_xyz_offset_keep_rpy(place_tilt, *REL_M5),   PLACE_MOVE_BASE5)

            # ---------------- PICK ----------------
            self._set_robot_status(3, f"타일 파지중 [{k+1}/{len(PLACE_PLAN_2x2)}]")
            movel(pick_above, vel=VELOCITY, acc=ACC)
            movel(pick_down,  vel=10, acc=10)
            self.gripper.set_width(CLOSE_W)
            wait(0.3)
            movel(pick_above, vel=VELOCITY, acc=ACC)
            movel(pick_move,  vel=VELOCITY, acc=ACC)

            # ---------------- PLACE ----------------
            self._set_robot_status(4, f"타일 배치중 [{k+1}/{len(PLACE_PLAN_2x2)}]")
            movel(place_tilt, vel=VELOCITY, acc=ACC)
            movel(place_down, vel=5, acc=5)

            # 슬라이드(정렬)
            movel(place_m1, vel=10, acc=10)
            movel(place_m2, vel=10, acc=10)

            # 힘제어로 타일 고정 (슬라이드 후, 오픈 전에)
            hold_with_force_4n(hold_n=4.0, timeout_s=3.0)

            # 릴리즈
            self.gripper.set_width(OPEN_W)
            time.sleep(0.3)

            # 후퇴
            movel(place_m3, vel=10, acc=10)
            # movel(place_m4, vel=10, acc=10)
            # movel(place_m5, vel=10, acc=10)
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
