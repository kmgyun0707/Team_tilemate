# ============================================================
# M0609 Tile Pick&Place 템플릿 (posx + RG2 Modbus width 제어)
# - PICK/PLACE 좌표: base_posx + (i * dy) 로 자동 생성
# - 첫 타일(i==0)만 "조심 모드" (저속/홀드 길게)
# - 모듈화: 외부에서 run_tile_pick_place() 함수로 호출 가능
# - Firebase 단계 업데이트 통합 (PICK=Step3, PLACE=Step4)
# ============================================================

import rclpy
import DR_init
import time
from pymodbus.client import ModbusTcpClient

# Firebase 임포트 (단계 업데이트용)
try:
    from firebase_admin import db
    FIREBASE_AVAILABLE = True
except ImportError:
    FIREBASE_AVAILABLE = False
    print("Warning: firebase_admin not available, step updates will be skipped")

# ----------------------------
# 로봇 기본 설정
# ----------------------------
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP  = "GripperDA_v1"

# 로봇 이동 속도/가속도 (두산 API movej/movel에 그대로 전달)
VELOCITY = 30
ACC      = 30

# DR_init에 로봇 ID/모델 등록 (dsr 라이브러리에서 사용)
DR_init.__dsr__id    = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# ----------------------------
# RG2 Modbus (그리퍼) 통신 설정
# ----------------------------
USE_GRIPPER = False  # ★ 그리퍼 사용 여부 (False = 동작만 테스트)
IP = "192.168.1.1"
PORT = 502
DEVICE = 65  # Modbus slave id (장비 설정에 따라 다를 수 있음)

# RG2 레지스터 주소 (너가 쓰던 주소 그대로)
REG_FORCE  = 0x0000  # 목표 힘 (write)
REG_WIDTH  = 0x0001  # 목표 폭 (write)
REG_START  = 0x0002  # 동작 시작 트리거 (write)
REG_STATUS = 0x010C  # 상태(예: busy bit) (read)
REG_ACT_W  = 0x0113  # 실제 폭(with offset) (read) - 필요 시 확인용

# ----------------------------
# 그리퍼 목표 폭 (m 단위)
# ----------------------------
# 예) 0.025 = 25mm, 0.015 = 15mm
OPEN_W  = 0.025
CLOSE_W = 0.015

# ----------------------------
# 타일 개수 / 그리드 간격 설정
# ----------------------------
NUM_TILES = 15

# PICK 좌표(기준점) + Y 간격(mm)
# - pick_above: 타일 위 안전 높이
# - pick_down : 집는 높이(접촉 구간)
PICK_ABOVE_BASE = [325, 18, 200, 12, -180, 102]
PICK_DOWN_BASE  = [325, 18, 165, 168, -180, -102]
PICK_MOVE_BASE  = [390, 25, 200, 168, -180, -102]
PICK_DY_MM      = 30.0  # 타일 한 칸씩 이동할 때 Y 증가량(mm)

# PLACE 좌표(기준점) + Y 간격(mm)
# - place_tilt: 45도 접근 자세
# - place_down: 모서리 접촉 하강 자세
# - place_move: 살짝 밀기(슬라이드) 자세
PLACE_TILT_BASE = [390, 180, 200, 103, 173, -163]
PLACE_DOWN_BASE = [390, 180, 170,  87, 125,  178]
PLACE_MOVE_BASE1 = [390, 180, 150,  87, 125,  178]
PLACE_MOVE_BASE2 = [390, 170, 145,  87, 125,  178]
PLACE_MOVE_BASE3 = [390, 150, 145,  87, 125,  178]
PLACE_MOVE_BASE4 = [390, 175, 135,  87, 125,  178]
PLACE_DY_MM     = -65.0  # 배치 한 칸씩 이동할 때 Y 변화량(mm) (방향에 따라 +/-)

def update_robot_step(step_number):
    """Firebase에 현재 공정 단계를 업데이트 (웹 UI 빨간불 제어)"""
    if FIREBASE_AVAILABLE:
        try:
            ref = db.reference('/robot_status')
            ref.update({'current_step': step_number})
        except Exception as e:
            print(f"Warning: Failed to update Firebase step: {e}")

def initialize_robot():
    """
    Tool/TCP 설정.
    - set_tool/set_tcp는 보통 MANUAL에서 설정 후 AUTONOMOUS로 복귀하는게 안전
    """
    from DSR_ROBOT2 import (
        set_tool, set_tcp,
        ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS,
        set_robot_mode
    )
    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(1)  # 설정 적용 안정화 대기

def set_gripper(width_m: float, force_n: float = 40.0):
    """
    RG2 폭(width) 제어 (Modbus 레지스터 직접 제어)
    - USE_GRIPPER가 False면 스킵
    """
    if not USE_GRIPPER:
        print(f"[GRIPPER DISABLED] Skipping gripper command: width={width_m*1000:.1f}mm")
        time.sleep(0.3)  # 그리퍼 동작 시간 시뮬레이션
        return
    
    # 실제 그리퍼 제어 (USE_GRIPPER = True일 때만)
    client = ModbusTcpClient(IP, port=PORT)
    if not client.connect():
        raise RuntimeError("Modbus 연결 실패")

    try:
        # force 변환: 40N -> 400 (0.1N 단위 가정)
        force_val = int(force_n * 10)

        # width 변환: (너 환경에서 측정한) 30mm -> 386 기준으로 비례 환산
        width_mm  = width_m * 1000.0
        width_val = int(width_mm * (386 / 30.0))

        # 1) 힘
        client.write_register(address=REG_FORCE, value=force_val, device_id=DEVICE)
        # 2) 폭
        client.write_register(address=REG_WIDTH, value=width_val, device_id=DEVICE)
        # 3) 시작
        client.write_register(address=REG_START, value=1, device_id=DEVICE)

        # 4) busy 종료 대기 (LSB=busy 가정)
        while True:
            rr = client.read_holding_registers(address=REG_STATUS, count=1, device_id=DEVICE)
            status = rr.registers[0]
            busy = status & 0b1
            if not busy:
                break
            time.sleep(0.05)

    finally:
        client.close()

def add_y_offset(posx, dy_mm: float):
    """
    posx에서 Y만 dy_mm만큼 이동한 새 posx를 생성
    - posx = [x, y, z, rx, ry, rz]
    """
    p = posx[:]          # 원본 보호용 복사
    p[1] = p[1] + dy_mm  # Y 축만 이동
    return p

def run_tile_pick_place(node, num_tiles=15):
    """
    외부에서 호출 가능한 타일 픽앤플레이스 함수
    
    Args:
        node: ROS2 노드 (로깅용)
        num_tiles: 작업할 타일 개수 (기본값 15)
    """
    from DSR_ROBOT2 import movej, movel, wait

    # 시작 안전 자세(관절)
    JReady = [0, 0, 90, 0, 90, 90]
    movej(JReady, vel=VELOCITY, acc=ACC)

    # 시작 오픈(그리퍼 벌려두기)
    node.get_logger().info("RG2 OPEN - init")
    set_gripper(OPEN_W)
    time.sleep(0.5)

    # 타일 개수만큼 반복
    for i in range(num_tiles):

        # -----------------------
        # 첫 타일만 "조심 모드"
        # -----------------------
        if i == 0:
            # 픽업 하강 저속
            v_pick_down, a_pick_down = 8, 8
            # 배치 하강 저속 (깨짐/충돌 방지)
            v_place_down, a_place_down = 3, 3
            # 슬라이드도 천천히
            v_slide, a_slide = 10, 10
            # 오픈 후 안정화 홀드 길게
            open_hold = 1.0
        else:
            # 이후 타일은 속도 좀 올려서 빠르게
            v_pick_down, a_pick_down = 12, 12
            v_place_down, a_place_down = 8, 8
            v_slide, a_slide = VELOCITY, ACC
            open_hold = 0.3

        # -----------------------
        # 좌표 생성 (base + i*dy)
        # -----------------------
        pick_dy  = i * PICK_DY_MM
        place_dy = i * PLACE_DY_MM

        pick_above = add_y_offset(PICK_ABOVE_BASE, pick_dy)
        pick_down  = add_y_offset(PICK_DOWN_BASE,  pick_dy)
        pick_move  = add_y_offset(PICK_MOVE_BASE,  pick_dy)

        place_tilt = add_y_offset(PLACE_TILT_BASE, place_dy)
        place_down = add_y_offset(PLACE_DOWN_BASE, place_dy)
        place_move1 = add_y_offset(PLACE_MOVE_BASE1, place_dy)
        place_move2 = add_y_offset(PLACE_MOVE_BASE2, place_dy)
        place_move3 = add_y_offset(PLACE_MOVE_BASE3, place_dy)
        place_move4 = add_y_offset(PLACE_MOVE_BASE4, place_dy)

        # -----------------------
        # PICK 시퀀스 → Step 3
        # -----------------------
        node.get_logger().info(f"[{i+1}/{num_tiles}] PICK - Step 3")
        update_robot_step(3)  # ★ Firebase Step 3 업데이트
        
        # 1) 타일 위로 접근
        movel(pick_above, vel=VELOCITY, acc=ACC)
        # 2) 집는 높이까지 하강(조심/빠름 모드에 따라 속도 다름)
        movel(pick_down,  vel=v_pick_down, acc=a_pick_down)
        # 3) 닫기
        set_gripper(CLOSE_W)
        wait(0.5)  # 물고 안정화
        # 4) 다시 상부로 상승
        movel(pick_above, vel=VELOCITY, acc=ACC)

        # 5) 타일 상부로 이동
        movel(pick_move, vel=VELOCITY, acc=ACC)

        # -----------------------
        # PLACE 시퀀스 → Step 4
        # -----------------------
        node.get_logger().info(f"[{i+1}/{num_tiles}] PLACE - Step 4")
        update_robot_step(4)  # ★ Firebase Step 4 업데이트
        
        # 1) 45도 접근 자세
        movel(place_tilt, vel=VELOCITY, acc=ACC)
        # 2) 하강(조심/빠름 모드)
        movel(place_down, vel=v_place_down, acc=a_place_down)
        # 3) 슬라이드(밀어서 안착)
        movel(place_move1, vel=v_slide, acc=a_slide)

        # 4) 슬라이드(밀어서 안착)
        movel(place_move2, vel=v_slide, acc=a_slide)

        # 5) 열기(릴리즈)
        set_gripper(OPEN_W)
        time.sleep(open_hold)

        # 6) 슬라이드(그리퍼 후퇴이동)
        movel(place_move3, vel=v_slide, acc=a_slide)
        movel(place_move4, vel=v_slide, acc=a_slide)
        movel(place_move3, vel=v_slide, acc=a_slide)

        # 7) 45도 자세로 복귀
        movel(place_tilt, vel=VELOCITY, acc=ACC)

    # 종료 후 안전 자세 복귀
    movej(JReady, vel=VELOCITY, acc=ACC)
    node.get_logger().info(f"Tile pick & place completed: {num_tiles} tiles")

def main(args=None):
    """
    ROS2 노드 생성 -> DR_init 노드 연결 -> 초기화 -> 작업 실행
    (독립 실행용)
    """
    rclpy.init(args=args)
    node = rclpy.create_node("tile_pick_place_grid", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    initialize_robot()
    run_tile_pick_place(node, num_tiles=NUM_TILES)

    rclpy.shutdown()

if __name__ == "__main__":
    main()