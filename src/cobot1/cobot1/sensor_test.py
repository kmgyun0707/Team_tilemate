import rclpy
import DR_init
import time
import os

# 로봇 설정 상수
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def print_colored(text, color_code):
    """터미널 출력을 보기 좋게 하기 위한 함수"""
    print(f"\033[{color_code}m{text}\033[0m")

def read_sensor_data():
    """로봇의 다양한 센서 데이터를 읽어오는 함수"""
    
    # 데이터를 읽어오는 함수들은 여기서 import 합니다.
    from DSR_ROBOT2 import (
        get_current_posj,       # 현재 관절 각도 (Joint Position) 
        get_current_posx,       # 현재 직교 좌표 (Task/TCP Position)
        get_external_torque,    # 외부 토크 (충돌 감지 등에 사용)
        get_tool_digital_input, # 툴 플랜지 디지털 입력 (그리퍼 센서 확인용)
        get_robot_mode,          # 로봇 모드 확인
        get_solution_space
    )

    print("Starting Sensor Monitoring... (Press Ctrl+C to stop)")
    
    try:
        while True:
            # 1. 현재 화면 지우기 (데이터가 갱신되는 것처럼 보이게 함)
            # os.system('clear') # 윈도우라면 'cls'
            
            print("=" * 60)
            print(f" [ Sensor Data Monitor - {ROBOT_MODEL} ]")
            print("=" * 60)

            # 2. 현재 관절 각도 (Joint Position) 읽기 [J1, J2, J3, J4, J5, J6]
            # 단위: degree
            current_joint = get_current_posj()
            print_colored(f"1. Current Joint (deg):", "32") # 초록색
            print(f"   {current_joint}") 

            # 3. 현재 TCP 좌표 (Task Position) 읽기 [X, Y, Z, A, B, C,솔루션 ]
            # 단위: mm, degree
            # RG2 그리퍼 길이를 포함한 끝단 좌표입니다 (initialize에서 set_tcp가 되었다면)
            current_posx = get_current_posx()
            print_colored(f"2. Current TCP Pose (mm, deg):", "34") # 파란색
            print(f"   {current_posx}")
            
            # 솔루션 스페이스 (관절 설정) 확인
            sol_space = get_solution_space(current_joint)
            print(f"   (Solution Space: {sol_space})") #2

            # 4. 외부 토크 (External Torque) 읽기
            # 로봇이 힘을 받고 있는지 확인할 때 유용합니다. [J1, J2, J3, J4, J5, J6]
            ext_torque = get_external_torque()
            print_colored(f"3. External Torque (Nm):", "33") # 노란색
            print(f"   {ext_torque}")

            # 5. 그리퍼(RG2) 센서 확인용 (옵션)
            # RG2가 툴 플랜지 I/O에 연결되어 있다면, 신호를 읽을 수 있습니다.
            # 보통 RG2는 내부적으로 통신하지만, 단순 배선일 경우 디지털 인풋을 확인합니다.
            print_colored(f"4. Tool Flange Digital Input:", "36") # 청록색
            try:
                # 툴 플랜지의 1번~6번 핀 상태를 읽어봅니다. (설치 방식에 따라 다름)
                input_1 = get_tool_digital_input(1)
                input_2 = get_tool_digital_input(2)
                print(f"   Input 1: {input_1} | Input 2: {input_2}")
            except Exception:
                print("   (Unable to read Tool I/O - might be disabled or irrelevant)")

            print("-" * 60)
            print("Press Ctrl+C to exit.")
            
            # 0.5초 대기 (너무 빠르게 출력되면 보기 힘듦)
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nMonitoring stopped by user.")

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("sensor_monitor", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        # 센서 데이터 읽기 루프 실행
        read_sensor_data()

    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()