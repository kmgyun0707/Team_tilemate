#!/usr/bin/env python3
# test_measure_tilt.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import DR_init
import time
import math

# ==========================================
# 1. 로봇 설정 상수
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
# 3. 로봇 초기화 함수
# ==========================================
def initialize_robot(node: Node):
    from DSR_ROBOT2 import set_tool, set_tcp, get_tool, get_tcp, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS
    from DSR_ROBOT2 import get_robot_mode, set_robot_mode

    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(2)
    
    node.get_logger().info("Initializing robot complete.")

# ==========================================
# 4-2. 국소적 요철 감지 및 스마트 압착 테스트 (조인트 비비기 적용)
# ==========================================
def test_local_compliance_compaction(node: Node, gripper: _GripperClient):
    # 🌟 amovej, get_current_posj 추가됨 (관절 제어용)
    from DSR_ROBOT2 import (
        posx, movel, movej, wait, get_current_posx, get_current_posj, amovej,
        task_compliance_ctrl, set_desired_force, check_force_condition,
        release_force, release_compliance_ctrl, set_ref_coord, get_tool_force,
        DR_BASE, DR_TOOL, DR_FC_MOD_REL, DR_AXIS_Z
    )

    node.get_logger().info("=== [실전 시퀀스: 9번 타일 스마트 압착 (관절 제어)] ===")
    JReady = [0, 0, 90, 0, 90, 0]
    movej(JReady, vel=VELOCITY, acc=ACC)
    wait(1.0)

    TOOL_GRIP_ABOVE = posx([531.2, -101.3 -87, 220.0, 169.29, 177.87, 169.98])
    TOOL_GRIP_DOWN  = posx([531.2, -101.3 -87, 146.0, 169.29, 177.87, 169.98])
    TOOL_WAYPOINT   = posx([470.0, 24.0, 230.0, 6.0, -179.0, 97.0])
    
    tool_rx, tool_ry, tool_rz = 75.0, 178.60, 77.00
    tile9_safe_pos = posx([539.05, 20.33, 230.0, tool_rx, tool_ry, tool_rz])

    OPEN_W, CLOSE_W  = 0.040, 0.005

    # 1. 툴 파지 및 이동
    node.get_logger().info("🛠️ 압착 툴을 파지하고 타일로 이동합니다...")
    gripper.set_width(OPEN_W)
    wait(1.0)
    movel(TOOL_GRIP_ABOVE, vel=VELOCITY, acc=ACC, ref=DR_BASE)
    movel(TOOL_GRIP_DOWN, vel=VELOCITY, acc=ACC, ref=DR_BASE)
    wait(0.5)
    gripper.set_width(CLOSE_W)
    wait(1.5)
    movel(TOOL_GRIP_ABOVE, vel=VELOCITY, acc=ACC, ref=DR_BASE)
    movel(TOOL_WAYPOINT, vel=VELOCITY, acc=ACC, ref=DR_BASE)
    movel(tile9_safe_pos, vel=VELOCITY, acc=ACC, ref=DR_BASE)
    wait(1.0)

    # 2. 압착 하강 준비
    set_ref_coord(DR_TOOL) 
    SMART_STIFFNESS = [3000, 3000, 20, 200, 200, 200]
    task_compliance_ctrl(stx=SMART_STIFFNESS, time=0.0)
    wait(0.2)

    # 🌟 10N의 부드러운 힘으로 타일 감지 하강!
    node.get_logger().info("   ⬇️ 10N의 힘으로 바닥을 향해 하강합니다.")
    set_desired_force(fd=[0, 0, 30.0, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

    t0 = time.time()
    touched = False

    while (time.time() - t0) < 10.0:
        if check_force_condition(DR_AXIS_Z, min=0, max=6.0) == -1:
            touched = True
            
            # 접촉 순간 데이터 기록
            contact_pos, _ = get_current_posx(DR_BASE)
            contact_z  = contact_pos[2]
            
            # 🌟 [핵심] 바닥에 닿은 순간의 '완벽한 모터(관절) 각도'를 저장!
            contact_joint = get_current_posj()

            node.get_logger().info("==================================================")
            node.get_logger().info(f" ✅ 바닥 접촉 감지 완료 (Z: {contact_z:.2f}mm)")
            node.get_logger().info("==================================================")
            # ... (앞부분 동일) ...
            set_desired_force(fd=[0, 0, 13.0, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL) # 회전하면서 바닥찍는 힘 
            
            from DSR_ROBOT2 import get_current_posj, amovej
            # 바닥에 닿은 순간의 '완벽한 모터(관절) 각도'를 저장!
            contact_joint = get_current_posj() 
            
            press_t0 = time.time()
            duration = 3.0          
            max_twist = 20.0        # 🌟 ±20도 회전 
            
            # 🌟 방향 전환을 위한 플래그 (1이면 오른쪽, -1이면 왼쪽)
            direction_flag = 1      
            last_switch_time = time.time()

            while (time.time() - press_t0) < duration:
                current_time = time.time()
                
                # 🌟 0.5초마다 방향을 반대로 바꿈! (진짜 와리가리)
                if current_time - last_switch_time > 0.5:
                    direction_flag *= -1  # 부호 반전! (1 -> -1 -> 1 ...)
                    last_switch_time = current_time
                
                # 1~5번 모터는 고정! 6번째 손목 모터에만 방향 플래그 적용
                target_joint = list(contact_joint)
                target_joint[5] = contact_joint[5] + (max_twist * direction_flag)
                
                # 조인트 비동기 제어
                amovej(target_joint, vel=80, acc=80) # 속도를 조금 더 올려서 빠릿하게!

                # 현재 힘 상태 실시간 감지
                from DSR_ROBOT2 import get_tool_force
                cur_force = get_tool_force(DR_BASE)
                fz_cur = abs(cur_force[2])
                
                # 로그에서 부호가 확실히 바뀌는지 확인 가능!
                node.get_logger().info(f"   ↔️ 비비는 중... 방향: {'우(Right)' if direction_flag == 1 else '좌(Left)'} | 목표 각도: {target_joint[5]:.1f}도 | Fz: {fz_cur:.1f}N")
                
                wait(0.1)

            # -----------------------------------------------------------------
            # 🌟 마무리: 다시 원래 수평으로 복귀 및 다짐!
            node.get_logger().info("   ✅ 비비기 완료. 원래 각도로 복귀하여 다짐(Tamping) 중...")
            
            # 수학 계산 없이, 처음 닿았던 완벽한 관절 상태로 깔끔하게 돌아옵니다.
            amovej(contact_joint, vel=40, acc=40)            
            wait(3.0) # 3초 대기

            final_pos, _ = get_current_posx(DR_BASE)
            node.get_logger().info("==================================================")
            node.get_logger().info("📊 [조인트 비비기 압착 최종 데이터]")
            node.get_logger().info(f" -> 접촉 시 높이: {contact_z:.2f} mm")
            node.get_logger().info(f" -> 최종 안착 높이: {final_pos[2]:.2f} mm")
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
    node.get_logger().warn("⚠️ 테스트 종료! 로봇이 툴을 파지하고 있습니다.")

import time
import math

def measure_local_tilt(node, p1, p2, p3, vel=30, acc=40):
    """
    3개의 안전 좌표(p1, p2, p3)를 받아 각각의 위치에서 바닥을 터치한 후,
    외적 알고리즘을 통해 해당 영역의 Roll, Pitch 기울기를 반환하는 함수.
    """
    from DSR_ROBOT2 import (
        movel, wait, get_current_posx,
        task_compliance_ctrl, set_desired_force, check_force_condition,
        release_force, release_compliance_ctrl, set_ref_coord,
        DR_BASE, DR_TOOL, DR_FC_MOD_REL, DR_AXIS_Z
    )

    node.get_logger().info("=== [국소 기울기 측정 알고리즘 시작] ===")

    probe_points = [p1, p2, p3]
    contact_positions = []
    
    for i, p_safe in enumerate(probe_points):
        node.get_logger().info(f"📍 Point {i+1} 탐색 위치로 이동 중...")
        movel(p_safe, vel=vel, acc=acc, ref=DR_BASE)
        wait(0.5)

        # 1. 툴 기준 컴플라이언스(순응) 켜기: Z축 강도를 20으로 확 낮춰서 깊게 내려갈 수 있도록 설정
        set_ref_coord(DR_TOOL)
        task_compliance_ctrl(stx=[3000, 3000, 20, 200, 200, 200], time=0.0)
        wait(0.2)
        
        # 2. amovel 없이 '힘'만으로 하강 유도
        node.get_logger().info("   바닥을 향해 부드럽게 하강합니다...")
        set_desired_force(fd=[0, 0, 30.0, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

        t0 = time.time()
        touched = False
        
        # 3. 바닥 감지 (8N 저항)
        while (time.time() - t0) < 15.0:
            if check_force_condition(DR_AXIS_Z, min=0, max=8.0) == -1:
                release_force() # 🚨 즉시 미는 힘 제거 (이게 바로 급정지 브레이크 역할!)
                release_compliance_ctrl() 
                
                # 안정화를 위한 짧은 대기 후 좌표 읽기
                wait(0.2)
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
            return None, None, None

        # 4. 안전 높이로 복귀
        node.get_logger().info("   안전 높이로 복귀합니다.")
        movel(p_safe, vel=vel, acc=acc, ref=DR_BASE) 
        wait(0.5)

    # 5. 수학 알고리즘: 3차원 벡터 외적을 이용한 기울기 도출
    cp1, cp2, cp3 = contact_positions
    vA = [cp2[0]-cp1[0], cp2[1]-cp1[1], cp2[2]-cp1[2]]
    vB = [cp3[0]-cp1[0], cp3[1]-cp1[1], cp3[2]-cp1[2]]
    
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

    # 측정된 기울기 각도와 터치한 좌표 3개를 반환
    return roll_deg, pitch_deg, contact_positions

# ==========================================
# 5. 메인 로직
# ==========================================
def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("measure_tilt_node", namespace=ROBOT_ID)

    DR_init.__dsr__node = node
    gripper = _GripperClient(node)

    try:
        initialize_robot(node)
        test_local_compliance_compaction(node, gripper)
    except KeyboardInterrupt:
        node.get_logger().info("\n강제 종료되었습니다.")
    except Exception as e:
        node.get_logger().error(f"오류 발생: {e}")
    finally:
        from DSR_ROBOT2 import release_force, release_compliance_ctrl
        try:
            release_force()
            release_compliance_ctrl()
        except:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()