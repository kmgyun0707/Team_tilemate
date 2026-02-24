#!/usr/bin/env python3
# tilemate_main/tile_motion_node.py
# 타일 9개 배치 작업을 수행하는 ROS2 노드
# 0223: 8개 타일 배치하는 코드 - 중간에 공중에서 떨어지는 타일 1개 #

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

OPEN_W  = 0.025
CLOSE_W = 0.005

# ----------------------------
# positions (absolute posx list)
# ----------------------------
pick_above = [344, -101, 300, 92, 179, 92]


# 집을 타일 위치(pick)
PICK_ABOVE_A = [344.54+5.0-2.0+8.0-8.0, -100.1-5.0-2.0+5.0, 260.95, 74.35, 178.8, 73.81]
PICK_ABOVE_B = [436.16+1.0-0.3, -98.84+2.0-0.1, 260.78, 69.2, 178.73, 68.63]
# 배치 타일 위치 
# PLACE_TILT_BASE01 = [402.08, 158.83, 229.77, 75.71, 178.59, 76.82]
# PLACE_TILT_BASE02 = [469.39, 157.97, 228.80,  69.56, 178.45, 70.97]
# PLACE_TILT_BASE03 = [539.79, 156.41, 229.13, 70.08, 178.43, 71.37]

# PLACE_TILT_BASE04 = [401.85, 90.05,  229.55, 83.17, 178.79, 84.01]
# PLACE_TILT_BASE05 = [469.94, 89.92,  228.78, 75.87, 178.68, 76.88]
# PLACE_TILT_BASE06 = [539.63, 88.61,  228.16, 71.45, 178.57, 72.51]

# PLACE_TILT_BASE07 = [401.19, 22.88,  228.09, 95.78, 178.83, 96.38]
# PLACE_TILT_BASE08 = [468.62, 21.5,   227.98, 85.68, 178.79, 86.43]
# PLACE_TILT_BASE09 = [539.05, 20.33,  227.54, 80.24, 178.71, 81.15]

PLACE_TILT_BASE01 = [402.08, 158.83, 229.77, 75.0, 178.60, 77.00]
PLACE_TILT_BASE02 = [469.39, 157.97, 228.80, 75.0, 178.60, 77.00]
PLACE_TILT_BASE03 = [539.79, 156.41, 229.13, 75.0, 178.60, 77.00]

PLACE_TILT_BASE04 = [401.85, 90.05,  229.55, 75.0, 178.60, 77.00]
PLACE_TILT_BASE05 = [469.94, 89.92,  228.78, 75.0, 178.60, 77.00]
PLACE_TILT_BASE06 = [539.63, 88.61,  228.16, 75.0, 178.60, 77.00]

PLACE_TILT_BASE07 = [401.19, 22.88,  228.09, 75.0, 178.60, 77.00]
PLACE_TILT_BASE08 = [468.62, 21.5,   227.98, 75.0, 178.60, 77.00]
PLACE_TILT_BASE09 = [539.05, 20.33,  227.54, 75.0, 178.60, 77.00]

pick_down  = [344, -101, 200, 50, 179, 140]
place_down = [401,  22, 170, 8, -179, 98]


# 압착판 도구 잡는 위치
TOOL_GRIP_ABOVE = [531.2, -101.3, 210, 169.29, 177.87, 169.98] # 흡착 도구 위치 (파지 준비 자세)
TOOL_GRIP_DOWN =  [531.2, -101.3, 165, 169.29, 177.87, 169.98]# 흡착 도구 위치 (파지 자세)
TOOL_WAYPOINT   = [470, 24, 230, 6, -179, 97]



OPEN_W  = 0.040  # 그리퍼 열림 (단위: m)
CLOSE_W = 0.005  # 그리퍼 닫힘 (타일 잡기)

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
        self._node.get_logger().info("[TOOL] (Grab)")
        self.set_width(CLOSE_W)
        time.sleep(1.0) # 그리퍼가 완전히 닫힐 때까지 잠시 대기
 
    def release(self):
        self._node.get_logger().info("[TOOL] (Release)")
        self.set_width(OPEN_W)
        time.sleep(1.0) # 그리퍼가 완전히 열릴 때까지 잠시 대기


class TileMotionNode(Node):
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
        self.pub_step = self.create_publisher(Int32, "/robot/step", 10)
        self.pub_completed_jobs = self.create_publisher(Int32, "/robot/completed_jobs", 10)
        self._completed_jobs = 0

        self._design_pattern = ['A'] * 9 # 기본값: 모두 A 패턴으로 시작 (필요시 /robot/design_ab 토픽으로 업데이트)

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

    def _cb_design_ab(self, msg: String):
        """ /robot/design_ab 토픽이 들어오면 호출되는 콜백 함수 """
        raw_string = msg.data
        self._design_pattern = [x.strip().upper() for x in raw_string.split(",")]
        self.get_logger().info(f"[TILE] 디자인 패턴 수신 및 파싱 완료: {self._design_pattern}")

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
                self.get_logger().warn("[TILE] stop_soft=True during pause -> abort")
                return

            self.get_logger().info(f"[TILE] run_once start token={tok}")
            self._perform_task_2x2()

            if not self._stop_soft:
                self._set_robot_status(5, "타일 작업 완료")

            self._publish_status(f"done:{tok}")

        except Exception as e:
            self.get_logger().error(f"[TILE] exception: {e}")
            self.get_logger().error(traceback.format_exc())
            self._publish_status(f"error:{tok}:{e}")

        finally:
            self._running = False

# 🚨 수정됨: tile_idx를 파라미터로 받아서 3의 배수인지 확인 + 실시간 각도 로그
#     def detach_tile(self, tile_idx):
#         self.get_logger().info(f"[TILE] Detaching tile {tile_idx} by tilting...")
#         from DSR_ROBOT2 import posx, amovel, wait, DR_TOOL, add_tcp, get_tcp, set_tcp, DR_BASE, set_robot_mode, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS, get_current_posx, check_motion
        
#         set_robot_mode(ROBOT_MODE_MANUAL)
#         tcp_name = "MySuction_v1"
#         tcp_offset = [0, 0, 265, 0, 0, 0] # 265 tcp 설정
#         add_tcp(tcp_name, tcp_offset)
#         set_tcp(tcp_name)
#         set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        
#         wait(0.5)
#         self.get_logger().info(f"[TILE] Current TCP: {get_tcp()}")
        
#         # ✅ 3의 배수(3, 6, 9) 자리에서는 특이점 회피를 위해 꺾는 각도를 -30으로 설정
#         if tile_idx % 3 == 0:
#             tilt_angle = -30
#         else:
#             tilt_angle = 30
            
#         tilt_forward = posx([0, 0, 0, 0, tilt_angle, 0])
        
#         # 🚨 [중요] movel 대신 amovel(비동기 이동)을 사용하여 이동 중에도 루프를 돌 수 있게 함
#         amovel(tilt_forward, vel=10, acc=10, ref=DR_TOOL, time=5.0)
        
#         # 이동하는 동안 현재 툴의 각도를 계속해서 출력
#         while check_motion() != 0:
#             # 현재 로봇의 Base 좌표계 기준 위치/각도를 가져옵니다.
#             cur_pos, _ = get_current_posx(DR_BASE)
#             # cur_pos = [X, Y, Z, A(Rx), B(Ry), C(Rz)]
#             rx, ry, rz = cur_pos[3], cur_pos[4], cur_pos[5]
            
#             # 보기 편하게 소수점 둘째 자리까지만 출력
#             self.get_logger().info(f"📐 [TILT_LOG] 현재 각도 - Rx: {rx:.2f}, Ry: {ry:.2f}, Rz: {rz:.2f}")
            
#             wait(0.1) # 0.1초마다 로그 출력
            
#             # 혹시 강제 중지 요청이 들어오면 루프 탈출
#             if self._stop_soft:
#                 break
        
#         wait(0.2)
        
#         set_robot_mode(ROBOT_MODE_MANUAL)
#         set_tcp("GripperDA_v1")
#         set_robot_mode(ROBOT_MODE_AUTONOMOUS)
#         wait(0.3)

    def detach_tile(self, tile_idx):
        self.get_logger().info(f"[TILE] Detaching tile {tile_idx} by tilting...")
        from DSR_ROBOT2 import posx, movel, wait, DR_TOOL, add_tcp, get_tcp, set_tcp, DR_BASE, set_robot_mode, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS
        
        set_robot_mode(ROBOT_MODE_MANUAL)
        tcp_name = "MySuction_v1"
        tcp_offset = [0, 0, 275, 0, 0, 0] 
        add_tcp(tcp_name, tcp_offset)
        set_tcp(tcp_name)
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        
        wait(0.5)
        
        # ✅ 방금 찾은 최적의 박리 각도 (25도에서 떨어지므로 여유있게 28도 셋팅)
        # 3의 배수(3, 6, 9) 자리에서는 특이점 회피를 위해 반대로 꺾음
        if tile_idx % 3 == 0:
            tilt_angle = -24
        else:
            tilt_angle = 24
            
        tilt_forward = posx([0, 0, 0, 0, tilt_angle, 0])
        
        # ✅ 최적 각도를 찾았으니, amovel 대신 movel로 복구하고 속도를 1.5초로 당겨서 스냅을 줍니다.
        movel(tilt_forward, vel=30, acc=30, ref=DR_TOOL, time=0.5)
        wait(0.2)
        
        set_robot_mode(ROBOT_MODE_MANUAL)
        set_tcp("GripperDA_v1")
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        wait(0.3)
    
    def _perform_task_2x2(self):
        from DSR_ROBOT2 import movej, movel, wait, posx, get_current_posx
        from DSR_ROBOT2 import DR_BASE

        def move_relative(dx: float, dy: float, dz: float) -> None :
            cur, _ = get_current_posx(DR_BASE)
            target = posx([cur[0] + dx, cur[1] + dy, cur[2] + dz, cur[3], cur[4], cur[5]])
            movel(target, vel=VELOCITY, acc=ACC)
        
        def compliant_approach(threshold_n=7.0, timeout_s=10.0):
            from DSR_ROBOT2 import (
                set_ref_coord, task_compliance_ctrl, set_desired_force,
                check_force_condition, release_force, release_compliance_ctrl,
                DR_TOOL, DR_FC_MOD_REL, DR_AXIS_Z
            )

            self.get_logger().info(f"[COMPLIANT] 힘 제어 하강 시작 (목표 힘: {threshold_n}N)")

            set_ref_coord(DR_TOOL)
            task_compliance_ctrl(stx=[3000, 3000, 80, 200, 200, 200])
            wait(0.5) 

            set_desired_force(
                fd=[0, 0, float(threshold_n+15), 0, 0, 0], 
                dir=[0, 0, 1, 0, 0, 0], 
                mod=DR_FC_MOD_REL
            )

            t0 = time.time()
            is_contact = False
            
            while True:
                ret = check_force_condition(DR_AXIS_Z, min=0, max=float(threshold_n))
                
                if ret == -1:
                    self.get_logger().info(f"✅ [COMPLIANT] 목표 힘({threshold_n}N) 도달! 하강 정지.")
                    is_contact = True
                    break
                
                wait(0.1)

            release_force()
            release_compliance_ctrl()
            wait(1.0)   
            set_ref_coord(DR_BASE)
            wait(1.0)

        # ============================================================
        # Task sequence 시작
        # ============================================================
        JReady = [0, 0, 90, 0, 90, 90]

        self.get_logger().info("[TILE] Move to JReady")
        movej(JReady, vel=VELOCITY, acc=ACC)

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
        # self.detach_tile(1)

        self.gripper.release()  
        movel(TOOL_GRIP_ABOVE, vel=VELOCITY, acc=ACC)
        movel(TOOL_GRIP_DOWN, vel=VELOCITY, acc=ACC)
        self.gripper.grab()
        
        movel(TOOL_GRIP_ABOVE, vel=VELOCITY, acc=ACC)
        
        self._set_robot_status(0, "안전 구역(Waypoint)으로 이동")
        movel(TOOL_WAYPOINT, vel=VELOCITY, acc=ACC)


        for tile_idx, place_pos in place_targets:
            
            list_index = tile_idx - 1
            
            if list_index < len(self._design_pattern):
                tile_type = self._design_pattern[list_index]
            else:
                tile_type = 'A'
            
            if tile_type == 'B':
                current_pick_pos = PICK_ABOVE_B
                color_name = "흰색"
            else:
                current_pick_pos = PICK_ABOVE_A
                color_name = "검정"

            # ---------------- PICK ----------------
            self._set_robot_status(1, f"타일 파지 준비 ({color_name} 타일함 상부) - {tile_idx}번 타일")
            movel(current_pick_pos, vel=VELOCITY, acc=ACC)

            self._set_robot_status(2, f"타일 파지 하강 ({tile_idx}번 타일)")
            compliant_approach(threshold_n=13.0, timeout_s=5.0)

            wait(0.3)
            self._set_robot_status(3, f"타일 파지 상승 ({tile_idx}번 타일)")
            movel(current_pick_pos, vel=VELOCITY, acc=ACC)


            wait(1.0)   
            self.get_logger().info(">>> moving +100Y")
            move_relative(0, 100, 0)

            # ---------------- PLACE ----------------
            self._set_robot_status(4, f"타일 배치 위치 상부 이동 ({tile_idx}번 타일)")
            movel(place_pos, vel=VELOCITY, acc=ACC)

            self._set_robot_status(4, f"타일 배치 하강 ({tile_idx}번 타일)")
            compliant_approach(threshold_n=11.0, timeout_s=10.0)

            # ---------------- DETACH ----------------
            # 🚨 수정됨: 현재 타일 번호를 넘겨줌
            self.detach_tile(tile_idx)

            self._set_robot_status(4, f"타일 배치 상부 복귀 ({tile_idx}번 타일)")
            movel(place_pos, vel=VELOCITY, acc=ACC)
            
            self.get_logger().info(f"🎉 {tile_idx}번 타일 작업 완료!")
        
    

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