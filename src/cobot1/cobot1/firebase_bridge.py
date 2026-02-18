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

def run_firebase_bridge():
    """로봇 센서 데이터를 읽어 터미널에 출력하고 Firebase로 전송하는 메인 루프"""
    
    # Firebase Admin SDK 초기화
    SERVICE_ACCOUNT_KEY_PATH = "/home/sa/cobot_ws/src/cobot1/config/co1-tiling-firebase-adminsdk-fbsvc-f4f88c3832.json" 
    DATABASE_URL = "https://co1-tiling-default-rtdb.asia-southeast1.firebasedatabase.app" 

    try:
        if not firebase_admin._apps:
            cred = credentials.Certificate(SERVICE_ACCOUNT_KEY_PATH)
            firebase_admin.initialize_app(cred, {'databaseURL': DATABASE_URL})
        ref = db.reference('/robot_status')
        print_colored("Firebase Connected Successfully!", "32")
    except Exception as e:
        print(f"Firebase Init Error: {e}")
        return

    # DSR_ROBOT2 함수들 임포트
    from DSR_ROBOT2 import (
        get_current_posj, get_current_posx, get_external_torque, get_robot_state
    )

    print("Starting Firebase Bridge & Monitoring... (Press Ctrl+C to stop)")
    
    try:
        while rclpy.ok():
            # 1. 로봇 데이터 읽기
            current_joint = get_current_posj()
            # posx는 ([x, y, z, a, b, c], sol) 형태의 튜플로 반환됨
            posx_raw = get_current_posx() 
            ext_torque = get_external_torque()
            robot_state = get_robot_state()

            # 2. 데이터 유효성 검사 및 추출
            # posx_raw가 튜플이므로 실제 좌표 리스트인 첫 번째 요소([0])를 가져옴
            if isinstance(posx_raw, tuple):
                posx_list = posx_raw[0]
            else:
                posx_list = posx_raw

            # 3. 터미널 출력
            print("=" * 50)
            print_colored(f"Joint (deg): {current_joint}", "32")
            print_colored(f"TCP Pose (mm): {posx_list[:3] if posx_list else 'N/A'}", "34")

            # 4. Firebase 데이터 업데이트
            if current_joint and posx_list:
                ref.update({
                    'completed_jobs': 0, 
                    'speed': round(current_joint[0], 1),
                    'collision_sensitivity': 7,
                    'state': "동작 중" if robot_state == 1 else "대기",
                    'current_step': 1,
                    # 리스트에서 개별 요소를 꺼내서 반올림
                    'pos_x': round(posx_list[0], 2),
                    'pos_y': round(posx_list[1], 2),
                    'pos_z': round(posx_list[2], 2)
                })
                print_colored(">>> Firebase Updated!", "36")

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nStopped by user.")

def main(args=None):
    rclpy.init(args=args)
    # 별도의 클래스 정의 없이 노드 생성 후 DR_init에 연결
    node = rclpy.create_node("firebase_bridge", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        # spin 대신 루프 함수 직접 실행
        run_firebase_bridge()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()