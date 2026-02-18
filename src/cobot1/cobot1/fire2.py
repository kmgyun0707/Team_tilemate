# firebase 연동 - 로봇 센서데이터 실시간 업데이트 및 웹 ui 제어

#!/usr/bin/env python3
import rclpy
import DR_init
import time
import firebase_admin
from firebase_admin import credentials, db

# 로봇 설정 상수
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def print_colored(text, color_code):
    """터미널 출력을 보기 좋게 하기 위한 함수"""
    print(f"\033[{color_code}m{text}\033[0m")

def wait_for_start():
    """웹에서 '시작' 버튼을 누를 때까지 대기"""
    print("Waiting for 'start' command from Web UI...")
    ref = db.reference('/robot_command/action')
    
    # 초기 상태 설정
    status_ref = db.reference('/robot_status')
    status_ref.update({'current_step': 0})
    
    while True:
        command = ref.get()
        if command == "start":
            print("Start command received! Beginning monitoring...")
            break
        elif command == "reset" or command == "idle":
            print(f"{command.capitalize()} state, waiting for start...")
            status_ref.update({'current_step': 0})
        time.sleep(0.5)

def run_firebase_bridge():
    """로봇 센서 데이터를 읽어 Firebase로 전송하는 메인 루프"""
    from DSR_ROBOT2 import (
        get_current_posj, get_current_posx, get_robot_state
    )

    # 시작 명령 대기
    wait_for_start()

    print("Starting Firebase Bridge & Monitoring... (Press Ctrl+C to stop)")
    ref = db.reference('/robot_status')
    
    try:
        while rclpy.ok():
            # 초기화 명령 체크
            cmd_ref = db.reference('/robot_command/action')
            if cmd_ref.get() == "reset":
                print("Reset command received! Waiting for restart...")
                ref.update({'current_step': 0})
                wait_for_start()
                continue
            
            # 일시정지 체크
            if cmd_ref.get() == "pause":
                print("Monitoring paused...")
                time.sleep(1)
                continue

            current_joint = get_current_posj()
            posx_raw = get_current_posx() 
            robot_state = get_robot_state()

            if isinstance(posx_raw, tuple):
                posx_list = posx_raw[0]
            else:
                posx_list = posx_raw

            # 터미널 출력 및 Firebase 데이터 업데이트 (current_step은 move_basic에서 제어함)
            if current_joint and posx_list:
                ref.update({
                    'speed': round(current_joint[0], 1),
                    'state': "동작 중" if robot_state == 1 else "대기",
                    'pos_x': round(posx_list[0], 2),
                    'pos_y': round(posx_list[1], 2),
                    'pos_z': round(posx_list[2], 2)
                })
                print_colored(f"Monitoring >>> TCP: {posx_list[:3]}", "36")

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nStopped by user.")

def main(args=None):
    # Firebase 초기화 (제일 먼저 수행)
    SERVICE_ACCOUNT_KEY_PATH = "/home/sa/cobot_ws/src/cobot1/config/co1-tiling-firebase-adminsdk-fbsvc-f4f88c3832.json" 
    DATABASE_URL = "https://co1-tiling-default-rtdb.asia-southeast1.firebasedatabase.app" 

    try:
        if not firebase_admin._apps:
            cred = credentials.Certificate(SERVICE_ACCOUNT_KEY_PATH)
            firebase_admin.initialize_app(cred, {'databaseURL': DATABASE_URL})
        print_colored("Firebase Connected Successfully!", "32")
    except Exception as e:
        print(f"Firebase Init Error: {e}")
        return

    rclpy.init(args=args)
    node = rclpy.create_node("firebase_bridge", namespace=ROBOT_ID)
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
        print_colored("Firebase initialized to idle state", "32")
        
        run_firebase_bridge()
    except KeyboardInterrupt:
        print("\nStopped by user.")
        # Ctrl+C 시 Firebase 초기화
        cmd_ref = db.reference('/robot_command')
        status_ref = db.reference('/robot_status')
        cmd_ref.set({'action': 'idle', 'timestamp': time.time() * 1000})
        status_ref.update({'current_step': 0, 'state': '대기'})
        print_colored("Firebase reset to idle state on exit", "33")
    except Exception as e:
        print(f"Error: {e}")
        # 에러 발생 시에도 Firebase 초기화
        try:
            cmd_ref = db.reference('/robot_command')
            status_ref = db.reference('/robot_status')
            cmd_ref.set({'action': 'idle', 'timestamp': time.time() * 1000})
            status_ref.update({'current_step': 0, 'state': '대기'})
            print_colored("Firebase reset to idle state on error", "33")
        except:
            pass
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()