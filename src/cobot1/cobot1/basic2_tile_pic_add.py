# 로봇 움직임 (타일 pick and place + firebase 통합) - tile_pic_place3_no_gripper 모듈 사용 버전(순응 힘 합치기 전 코드)

import rclpy
import DR_init
import time
import firebase_admin
from firebase_admin import credentials, db
from cobot1 import move_pause_resume # 로봇 제어 유틸리티 모듈 import

# 로봇팔용
from cobot1 import tile_pic_place3 as tile_pic_place3_module
from cobot1 import tile_pic_place3_no_gripper as tile_pic_place3_no_gripper_module

# 로봇 설정 상수
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

# 이동 속도 및 가속도
VELOCITY = 40
ACC = 60

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# --- [통합] Firebase 설정 (명령 수신 및 단계 업데이트용) ---
SERVICE_ACCOUNT_KEY_PATH = "/home/sa/cobot_ws/src/cobot1/config/co1-tiling-firebase-adminsdk-fbsvc-f4f88c3832.json" 
DATABASE_URL = "https://co1-tiling-default-rtdb.asia-southeast1.firebasedatabase.app" 

if not firebase_admin._apps:
    cred = credentials.Certificate(SERVICE_ACCOUNT_KEY_PATH)
    firebase_admin.initialize_app(cred, {'databaseURL': DATABASE_URL})

def update_robot_step(step_number):
    """Firebase에 현재 공정 단계를 업데이트 (웹 UI 빨간불 제어)"""
    ref = db.reference('/robot_status')
    ref.update({'current_step': step_number})

def wait_for_start():
    """웹에서 '시작' 버튼을 누를 때까지 대기"""
    print("Waiting for 'start' command from Web UI...")
    ref = db.reference('/robot_command/action')
    
    # 초기 상태 설정 (공정 단계 하이라이트 제거)
    status_ref = db.reference('/robot_status')
    status_ref.update({'current_step': 0})
    
    while rclpy.ok():
        command = ref.get()
        if command == "start":
            print("Start command received! Beginning task...")
            break
        elif command == "reset" or command == "idle":
            print(f"{command.capitalize()} command received, waiting for start...")
            status_ref.update({'current_step': 0})
        time.sleep(0.5)
# -------------------------------------------------------

def initialize_robot():
    """로봇의 Tool과 TCP를 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp, get_tool, get_tcp, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS
    from DSR_ROBOT2 import get_robot_mode, set_robot_mode

    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(2)
    print("#" * 50)
    print("Initializing robot with the following settings:")
    print(f"ROBOT_ID: {ROBOT_ID}")
    print(f"ROBOT_MODEL: {ROBOT_MODEL}")
    print(f"ROBOT_TCP: {get_tcp()}") 
    print(f"ROBOT_TOOL: {get_tool()}")
    print(f"ROBOT_MODE 0:수동, 1:자동 : {get_robot_mode()}")
    print("#" * 50)

def perform_task():
    """로봇이 수행할 작업 및 실시간 단계 보고 (동작 완료 확인 로직 통합)"""
    print("Performing task...")
    from DSR_ROBOT2 import posx, movej, movel, get_robot_state
    
    # 위치 설정
    JReady = [0, 0, 90, 0, 90, 0]
    JHome = [0, 0, 90, 0, 90, 0]  # 초기 위치 (Home)
    pos1 = posx([500, 80, 200, 150, 179, 150])
    
    completed_count = 0
    ref = db.reference('/robot_status')
    cmd_ref = db.reference('/robot_command/action')
    
    is_paused = False  # 일시정지 상태 추적

    while rclpy.ok() and completed_count < 45:  # 총 45개 타일
        # --- [1] 초기화 명령 체크 (루프 시작 시) ---
        if cmd_ref.get() == "reset":
            print("Reset command received! Moving to home position...")
            try:
                movej(JHome, vel=VELOCITY, acc=ACC)
                print("Robot moved to home position")
            except Exception as e:
                print(f"Error moving to home: {e}")
            
            completed_count = 0
            ref.update({'completed_jobs': 0, 'current_step': 0, 'working_tile': 0})
            is_paused = False
            wait_for_start()  # 다시 시작 명령 대기
            continue
        
        # --- [2] 일시정지 체크 및 서비스 호출 ---
        current_cmd = cmd_ref.get()
        if current_cmd == "pause" and not is_paused:
            print("Pause command received! Calling pause service...")
            if move_pause_resume.call_pause(ROBOT_ID):
                is_paused = True
                ref.update({'state': '일시정지'})
        
        # 일시정지 상태일 때 - 재개 명령 대기
        while is_paused and rclpy.ok():
            current_cmd = cmd_ref.get()
            if current_cmd == "start":
                print("Resume command received! Calling resume service...")
                if move_pause_resume.call_resume(ROBOT_ID):
                    is_paused = False
                    ref.update({'state': '동작 중'})
                    break
            
            if current_cmd == "reset":
                print("Reset command during pause! Moving to home position...")
                try:
                    movej(JHome, vel=VELOCITY, acc=ACC)
                except: pass
                completed_count = 0
                ref.update({'completed_jobs': 0, 'current_step': 0, 'working_tile': 0})
                is_paused = False
                wait_for_start()
                break
            time.sleep(0.3)
        
        if current_cmd == "reset":
            continue

        # --- [3] 실제 작업 시퀀스 시작 ---
        print(f"Executing cycle {completed_count + 1}/45...")
        tile_id = completed_count + 1
        ref.update({'working_tile': tile_id})
        
        # Step 1: 접착제 파지
        print("Step 1: Moving to JReady (접착제 파지)")
        update_robot_step(1)
        movej(JReady, vel=VELOCITY, acc=ACC)
        
        if cmd_ref.get() == "reset": continue

        # Step 2: 접착제 도포 (가상 공정)
        update_robot_step(2)
        time.sleep(1.0)
        
        if cmd_ref.get() == "reset": continue

        # Step 3 & 4: 타일 픽앤플레이스 작업
        # tile_pic_place3 내부에서 Step 3(PICK), Step 4(PLACE) 자동 업데이트
        print("Step 3 & 4: Tile Pick & Place")
        
        try:
            tile_pic_place3_no_gripper_module.run_tile_pick_place(
                node=DR_init.__dsr__node,  # ROS2 노드 전달
                num_tiles=1  # 한 번에 1개 타일만 작업
            )
        except Exception as e:
            print(f"Error during tile pick & place: {e}")
            ref.update({'state': '오류', 'error_message': str(e)})
            raise
        
        if cmd_ref.get() == "reset": continue
        
        # 사이클 완료 처리
        completed_count += 1
        ref.update({'completed_jobs': completed_count, 'working_tile': 0})
        print(f"Tile {completed_count} completed and updated to Firebase!")
        
        time.sleep(0.5)
    
    print("All 45 tiles completed!")
    
def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 동작 수행"""
    rclpy.init(args=args)
    node = rclpy.create_node("move_basic", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        # 프로그램 시작 시 Firebase 초기화
        cmd_ref = db.reference('/robot_command')
        status_ref = db.reference('/robot_status')
        cmd_ref.set({'action': 'idle', 'timestamp': time.time() * 1000})
        status_ref.update({
            'current_step': 0, 
            'state': '대기',
            'completed_jobs': 0  # 타일 진행도 초기화 추가
        })
        print("Firebase initialized to idle state")
        
        initialize_robot()
        
        # tile_pic_place3_no_gripper 모듈의 로봇 초기화도 실행
        tile_pic_place3_no_gripper_module.initialize_robot()
        
        wait_for_start() # 시작 명령 대기 추가
        perform_task()
    except KeyboardInterrupt:
        print("\nNode interrupted by user. Shutting down...")
        # Ctrl+C 시 Firebase 초기화
        cmd_ref = db.reference('/robot_command')
        status_ref = db.reference('/robot_status')
        cmd_ref.set({'action': 'idle', 'timestamp': time.time() * 1000})
        status_ref.update({'current_step': 0, 'state': '대기'})
        print("Firebase reset to idle state on exit")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        # 에러 발생 시에도 Firebase 초기화
        cmd_ref = db.reference('/robot_command')
        status_ref = db.reference('/robot_status')
        cmd_ref.set({'action': 'idle', 'timestamp': time.time() * 1000})
        status_ref.update({'current_step': 0, 'state': '대기'})
        print("Firebase reset to idle state on error")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()