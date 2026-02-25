#!/usr/bin/env python3
# test_measure_tilt.py
# 목적: 1, 3, 9번 타일 끝단 3점을 터치하여 전체 바닥의 기울기를 계산하는 알고리즘

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import DR_init
import time
import math

# ==========================================
# 1. 로봇 설정 상수 (기본 코드 기준)
# ==========================================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

VELOCITY = 30
ACC = 40

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# ==========================================
# 2. 그리퍼 제어 클라이언트
# ==========================================
class _GripperClient:
    def __init__(self, node: Node):
        self._node = node
        self._pub = node.create_publisher(Float64, "/gripper/width_m", 10)

    def set_width(self, width_m: float):
        msg = Float64()
        msg.data = float(width_m)
        self._pub.publish(msg)
        self._node.get_logger().info(f"[GRIPPER] width_m={msg.data:.4f}")

    def close_fully(self):
        self._node.get_logger().info("[GRIPPER] 툴 측정을 위해 그리퍼를 완전히 닫습니다.")
        self.set_width(0.0)
        time.sleep(1.0)


# ==========================================
# 3. 로봇 초기화 함수 (기본 코드 기준)
# ==========================================
def initialize_robot(node: Node):
    """로봇의 Tool과 TCP를 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp, get_tool, get_tcp, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS
    from DSR_ROBOT2 import get_robot_mode, set_robot_mode

    # Tool과 TCP 설정시 매뉴얼 모드로 변경해서 진행
    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(2)  # 설정 안정화를 위해 잠시 대기
    
    # 설정된 상수 출력 (Logger 사용)
    node.get_logger().info("#" * 50)
    node.get_logger().info("Initializing robot with the following settings:")
    node.get_logger().info(f"ROBOT_ID: {ROBOT_ID}")
    node.get_logger().info(f"ROBOT_MODEL: {ROBOT_MODEL}")
    node.get_logger().info(f"ROBOT_TCP: {get_tcp()}") 
    node.get_logger().info(f"ROBOT_TOOL: {get_tool()}")
    node.get_logger().info(f"ROBOT_MODE 0:수동, 1:자동 : {get_robot_mode()}")
    node.get_logger().info(f"VELOCITY: {VELOCITY}")
    node.get_logger().info(f"ACC: {ACC}")
    node.get_logger().info("#" * 50)

# ==========================================
# 4. 기울기 측정 알고리즘 함수
# ==========================================
def measure_global_tilt(node: Node, gripper: _GripperClient):
    from DSR_ROBOT2 import (
        posx, movel, movej, wait, get_current_posx,
        task_compliance_ctrl, set_desired_force, check_force_condition,
        release_force, release_compliance_ctrl, set_ref_coord,
        DR_BASE, DR_TOOL, DR_FC_MOD_REL, DR_AXIS_Z
    )

    node.get_logger().info("=== [기울기 측정 알고리즘 시작] ===")

    # 1. 안전한 홈 위치(JReady)로 정렬
    JReady = [0, 0, 90, 0, 90, 0] # 사용자의 JReady 좌표 반영
    node.get_logger().info("🚀 JReady(홈 위치)로 이동합니다...")
    movej(JReady, vel=VELOCITY, acc=ACC)
    wait(1.0)

    # 2. 그리퍼 완전히 닫기
    gripper.close_fully()
    wait(1.0)

    # 3. 3점 탐색 위치 정의 (Z=270 안전 높이)
    tool_rx, tool_ry, tool_rz = 75.0, 178.60, 77.00
    

    
    probe_points = [
        posx([402.08 - 30.0, 158.83 + 30.0, 200.0, tool_rx, tool_ry, tool_rz]), # P1 (1번 타일 좌측상단)
        posx([539.79 + 30.0, 156.41 + 30.0, 200.0, tool_rx, tool_ry, tool_rz]), # P2 (3번 타일 우측상단)
        posx([539.05 + 30.0,  20.33 - 30.0, 200.0, tool_rx, tool_ry, tool_rz])  # P3 (9번 타일 우측하단)
    ]
    
    contact_positions = []
    
    for i, p_safe in enumerate(probe_points):
        node.get_logger().info(f"📍 Point {i+1} 탐색 위치로 이동 중...")
        movel(p_safe, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        wait(0.5)

        # 1. 툴 기준 컴플라이언스(순응) 켜기: Z축 강도를 100으로 확 낮춰서 깊게 내려갈 수 있도록 설정
        set_ref_coord(DR_TOOL)
        task_compliance_ctrl(stx=[3000, 3000, 20, 200, 200, 200], time=0.0)
        wait(0.2)
        
        # 2. amovel 없이 '힘'만으로 하강 유도 (이동 거리 = 30N / 100(강도) = 300mm 이동 가능!)
        node.get_logger().info("   바닥을 향해 부드럽게 하강합니다...")
        set_desired_force(fd=[0, 0, 30.0, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

        t0 = time.time()
        touched = False
        
        # 3. 바닥 감지 (8N 저항)
        while (time.time() - t0) < 15.0:
            if check_force_condition(DR_AXIS_Z, min=0, max=8.0) == -1:
                release_force() # 🚨 즉시 미는 힘 제거 (이게 바로 급정지 브레이크 역할!)
                release_compliance_ctrl() 
                
                cur_pos, _ = get_current_posx(DR_BASE)
                contact_positions.append([cur_pos[0], cur_pos[1], cur_pos[2]])
                node.get_logger().info(f"   ✅ 바닥 감지 성공: Z = {cur_pos[2]:.2f} mm")
                touched = True
                break
            wait(0.05)
            
        if not touched:
            release_force()
            release_compliance_ctrl()
            node.get_logger().error(f"Point {i+1} 바닥 감지 실패 (타임아웃). 측정을 중단합니다.")
            return

        # 4. 안전 높이(Z=270)로 복귀
        node.get_logger().info("   안전 높이로 복귀합니다.")
        movel(p_safe, vel=VELOCITY, acc=ACC, ref=DR_BASE) 
        wait(0.5)

    # 5. 수학 알고리즘: 3차원 벡터 외적을 이용한 기울기 도출
    p1, p2, p3 = contact_positions
    vA = [p2[0]-p1[0], p2[1]-p1[1], p2[2]-p1[2]]
    vB = [p3[0]-p1[0], p3[1]-p1[1], p3[2]-p1[2]]
    
    Nx = vA[1]*vB[2] - vA[2]*vB[1]
    Ny = vA[2]*vB[0] - vA[0]*vB[2]
    Nz = vA[0]*vB[1] - vA[1]*vB[0]
    
    norm = math.sqrt(Nx**2 + Ny**2 + Nz**2)
    Nx, Ny, Nz = Nx/norm, Ny/norm, Nz/norm
    
    if Nz < 0: 
        Nx, Ny, Nz = -Nx, -Ny, -Nz
        
    roll_deg  = math.degrees(math.atan2(Ny, Nz))
    pitch_deg = math.degrees(math.atan2(-Nx, math.sqrt(Ny**2 + Nz**2)))
    
    node.get_logger().info("==================================================")
    node.get_logger().info("📊 [알고리즘 분석결과] 평면 기울기 도출 완료")
    node.get_logger().info(f" -> X축 방향 기울어짐 (Roll)  : {roll_deg:.3f} 도")
    node.get_logger().info(f" -> Y축 방향 기울어짐 (Pitch) : {pitch_deg:.3f} 도")
    node.get_logger().info("==================================================")

    # 테스트 종료 후 다시 JReady로 복귀
    node.get_logger().info("테스트 완료. JReady로 복귀합니다.")
    movej(JReady, vel=VELOCITY, acc=ACC)

# ==========================================
# 4-2. 국소적 요철 감지 및 스마트 압착 테스트
# ==========================================
# ==========================================
# 4-2. 국소적 요철 감지 및 스마트 압착 테스트 (실제 파지 시퀀스 포함)
# ==========================================
def test_local_compliance_compaction(node: Node, gripper: _GripperClient):
    from DSR_ROBOT2 import (
        posx, movel, wait, get_current_posx,
        task_compliance_ctrl, set_desired_force, check_force_condition,
        release_force, release_compliance_ctrl, set_ref_coord, movej, get_tool_force,
        DR_BASE, DR_TOOL, DR_FC_MOD_REL, DR_AXIS_Z, amovel
    )

    node.get_logger().info("=== [실전 시퀀스: 툴 파지 -> 웨이포인트 -> 9번 타일 스마트 압착] ===")
    JReady = [0, 0, 90, 0, 90, 0]
    node.get_logger().info("🚀 안전을 위해 JReady로 먼저 이동합니다...")
    movej(JReady, vel=VELOCITY, acc=ACC)
    wait(1.0)

    # 네 메인 코드의 좌표들 가져오기
    TOOL_GRIP_ABOVE = posx([531.2, -101.3 -87, 220.0, 169.29, 177.87, 169.98])
    TOOL_GRIP_DOWN  = posx([531.2, -101.3 -87, 146.0, 169.29, 177.87, 169.98])
    TOOL_WAYPOINT   = posx([470.0, 24.0, 230.0, 6.0, -179.0, 97.0])
    
    # 9번 타일 안전 접근 좌표 (Z=230에서 하강 시작)
    tool_rx, tool_ry, tool_rz = 75.0, 178.60, 77.00
    tile9_safe_pos = posx([539.05, 20.33, 230.0, tool_rx, tool_ry, tool_rz])

    # 그리퍼 폭 세팅값
    OPEN_W  = 0.040
    CLOSE_W = 0.005

    # --------------------------------------------------
    # 1. 툴 파지 시퀀스
    # --------------------------------------------------
    node.get_logger().info("🛠️ 압착 툴을 가지러 이동합니다...")
    
    # 그리퍼 열기
    gripper.set_width(OPEN_W)
    wait(1.0)

    # 툴 거치대 상단 이동
    movel(TOOL_GRIP_ABOVE, vel=VELOCITY, acc=ACC, ref=DR_BASE)
    
    # 툴 거치대 하강
    movel(TOOL_GRIP_DOWN, vel=VELOCITY, acc=ACC, ref=DR_BASE)
    wait(0.5)

    # 그리퍼 닫기 (파지)
    node.get_logger().info("   툴을 파지합니다.")
    gripper.set_width(CLOSE_W)
    wait(1.5)

    # 툴 들고 상승
    movel(TOOL_GRIP_ABOVE, vel=VELOCITY, acc=ACC, ref=DR_BASE)

    # --------------------------------------------------
    # 2. 웨이포인트(안전구역)를 거쳐 9번 타일로 이동
    # --------------------------------------------------
    node.get_logger().info("📍 안전 웨이포인트를 경유하여 9번 타일로 이동합니다...")
    movel(TOOL_WAYPOINT, vel=VELOCITY, acc=ACC, ref=DR_BASE)
    movel(tile9_safe_pos, vel=VELOCITY, acc=ACC, ref=DR_BASE)
    wait(1.0)

    # --------------------------------------------------
    # 3. 스마트 압착 (발목 꺾기 및 나선형 비비기) 진행
    # --------------------------------------------------
    node.get_logger().info("   [스마트 압착] 바닥 기준 수직 하강으로 똑바로 내려갑니다...")
    
    set_ref_coord(DR_TOOL) 
    
    SMART_STIFFNESS = [3000, 3000, 20, 200, 200, 200]
    task_compliance_ctrl(stx=SMART_STIFFNESS, time=0.0)
    wait(0.2)

    # 🌟 2. Base 기준 Z축 아래 방향(-1)으로 30N의 힘을 가하여 완벽한 수직 하강 유도
    set_desired_force(fd=[0, 0, 30.0, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

    t0 = time.time()
    touched = False

    # 바닥 감지 및 실시간 평탄화(힘/각도) 모니터링
    while (time.time() - t0) < 10.0:
        # Base 기준이므로 밀리는 힘(반발력)을 감지.
        if check_force_condition(DR_AXIS_Z, min=0, max=6.0) == -1:
            touched = True

            # 접촉하는 순간의 모든 정보를 기준점으로 저장 (영점)
            contact_pos, _ = get_current_posx(DR_BASE)
            contact_rx = contact_pos[3]
            contact_ry = contact_pos[4]
            contact_z  = contact_pos[2]

            init_force = get_tool_force(DR_BASE)
            fz_init = abs(init_force[2])

            node.get_logger().info("==================================================")
            node.get_logger().info(f" ✅ 바닥 접촉 감지 완료 (Z: {contact_z:.2f}mm)")
            node.get_logger().info(f" 📍 압착 시작 기준점 설정: Rx={contact_rx:.2f}, Ry={contact_ry:.2f}")
            node.get_logger().info(f" 💥 초기 접촉 힘: {fz_init:.2f} N")
            node.get_logger().info("==================================================")

            # 🌟 [오류 완벽 해결] 카테시안(XYZ) 제어를 버리고 조인트(관절) 제어로 변경!
            node.get_logger().info("   ↔️ 제자리 비틀기 압착 시작: 시멘트를 25도로 회전하며 폅니다.")
            node.get_logger().info("   ⬇️ 비비기 저항을 줄이기 위해 누르는 힘을 5N으로 낮춥니다.")
            set_desired_force(fd=[0, 0, 5.0, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            
            from DSR_ROBOT2 import get_current_posj, amovej
            # 바닥에 닿은 순간의 '완벽한 관절 상태'를 기억해둡니다.
            contact_joint = get_current_posj() 
            
            press_t0 = time.time()
            duration = 5.0          # 비비기 시간
            max_twist = 25.0        # 🌟 귀엽게 비빌 각도! (±25도)
            wobble_speed = 1.5      # 1초에 1.5번 왕복
            
            while (time.time() - press_t0) < duration:
                elapsed = time.time() - press_t0
                
                # 다른 관절은 모두 고정! 오직 6번째 손목 관절(인덱스 5)만 sin 파형으로 흔듭니다.
                target_joint = list(contact_joint)
                target_joint[5] = contact_joint[5] + max_twist * math.sin(2 * math.pi * wobble_speed * elapsed)
                
                # 조인트 비동기 제어로 부드럽고 찰지게 와리가리
                amovej(target_joint, vel=60, acc=60)

                # 현재 힘 상태 실시간 감지
                from DSR_ROBOT2 import get_tool_force
                cur_force = get_tool_force(DR_BASE)
                fz_cur = abs(cur_force[2])
                
                node.get_logger().info(f"   ↔️ 귀엽게 비비는 중... 손목 각도: {target_joint[5]:.1f}도 | Fz: {fz_cur:.1f}N")
                
                wait(0.1)

            # -----------------------------------------------------------------
            # 최종 정렬 및 마무리
            # -----------------------------------------------------------------
            node.get_logger().info("   ✅ 비비기 완료. 최종 수평 정렬 및 다짐(Tamping) 중...")
            
            # 수학 계산 없이, 처음 바닥에 닿았을 때의 완벽한 관절 상태로 쓱 되돌아갑니다.
            amovej(contact_joint, vel=40, acc=40)
            wait(0.5)
            
            node.get_logger().info("   ⬆️ 최종 안착을 위해 누르는 힘을 다시 10N으로 올립니다.")
            set_desired_force(fd=[0, 0, 10.0, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            
            wait(2.0) # 시멘트가 수평으로 쫙 펴지고 자리잡을 때까지 2초 대기

            





            final_pos, _ = get_current_posx(DR_BASE)
            node.get_logger().info("==================================================")
            node.get_logger().info("📊 [나선형 압착 최종 데이터 분석]")
            node.get_logger().info(f" -> 접촉 시 높이: {contact_z:.2f} mm")
            node.get_logger().info(f" -> 최종 안착 높이: {final_pos[2]:.2f} mm")
            node.get_logger().info(f" -> 최종 안착 각도: Rx={final_pos[3]:.2f}, Ry={final_pos[4]:.2f}")
            node.get_logger().info(f" -> 💡 총 압착 깊이: {contact_z - final_pos[2]:.2f} mm")
            node.get_logger().info("==================================================")
            
            release_force()
            release_compliance_ctrl()
            break
        wait(0.05)

    if not touched:
        release_force()
        release_compliance_ctrl()
        node.get_logger().error("바닥 감지 실패 (타임아웃).")

    node.get_logger().info("   안전 높이로 복귀합니다.")
    movel(tile9_safe_pos, vel=VELOCITY, acc=ACC, ref=DR_BASE)
    wait(0.5)

    node.get_logger().warn("⚠️ 테스트 종료! 로봇이 툴을 파지하고 있습니다. 툴 거치대로 수동 복귀시키거나 툴을 빼주세요.")
# ==========================================
# 5. 메인 로직
# ==========================================
def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("measure_tilt_node", namespace=ROBOT_ID)

    # DR_init에 노드 설정
    DR_init.__dsr__node = node
    gripper = _GripperClient(node)

    try:
        # 1. 로봇 툴/TCP 설정 초기화 (한 번만 수행)
        initialize_robot(node)

        # 2. 기울기 측정 수행
        # measure_global_tilt(node, gripper)
        # 3. 국소적 요철 감지 및 스마트 압착 테스트
        test_local_compliance_compaction(node, gripper)

    except KeyboardInterrupt:
        node.get_logger().info("\n사용자에 의해 강제 종료되었습니다.")
    except Exception as e:
        node.get_logger().error(f"예기치 않은 오류 발생: {e}")
    finally:
        from DSR_ROBOT2 import release_force, release_compliance_ctrl
        try:
            # 비상 상황에서도 힘 제어가 물려있지 않게 확실히 해제
            release_force()
            release_compliance_ctrl()
        except:
            pass
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()