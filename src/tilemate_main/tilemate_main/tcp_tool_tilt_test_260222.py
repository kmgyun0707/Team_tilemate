# 타일 적용 테스트 코드
# 1. TCP 기준으로 기울이기 (Pitch, Roll)
# 2. TCP 기준으로 특정 좌표로 이동 (DR_TOOL 참조)

import rclpy
import DR_init
import time

# 로봇 설정 상수 (필요에 따라 수정)
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

# 이동 속도 및 가속도 (필요에 따라 수정)
VELOCITY = 20
ACC = 20

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def initialize_robot():
    """로봇의 Tool과 TCP를 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp,get_tool,get_tcp,ROBOT_MODE_MANUAL,ROBOT_MODE_AUTONOMOUS,add_tcp,get_current_posx  # 필요한 기능만 임포트
    from DSR_ROBOT2 import get_robot_mode,set_robot_mode

    # Tool과 TCP 설정시 매뉴얼 모드로 변경해서 진행
    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    tcp_name = "MySuction_v1"
    tcp_offset = [0, 0, 260, 0, 0, 0] # x, y, z, a, b, c
    
    # ConfigCreateTcp 대신 config_create_tcp를 사용하세요.
    cur_tcp=get_tcp()
    print(f"현재 TCP: {cur_tcp}")
    add_tcp(tcp_name, tcp_offset) 
    set_tcp(tcp_name) # 생성한 TCP를 현재 활성화
    cur_tcp=get_tcp()
    print(f"현재 TCP: {cur_tcp}")
    
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(2)  # 설정 안정화를 위해 잠시 대기
    # 설정된 상수 출력
    print("#" * 50)
    print("Initializing robot with the following settings:")
    print(f"ROBOT_ID: {ROBOT_ID}")
    print(f"ROBOT_MODEL: {ROBOT_MODEL}")
    print(f"ROBOT_TCP: {get_tcp()}") 
    print(f"ROBOT_TOOL: {get_tool()}")
    print(f"ROBOT_MODE 0:수동, 1:자동 : {get_robot_mode()}")
    print(f"VELOCITY: {VELOCITY}")
    print(f"ACC: {ACC}")
    print("#" * 50)



def perform_task():
    """로봇이 수행할 작업"""
    print("Performing task...")
    # DR_TOOL 추가 Import 필수
    from DSR_ROBOT2 import posx, movej, movel, set_ref_coord, wait, DR_TOOL,add_tcp,get_tcp,set_tcp,DR_BASE,DR_WORLD,amovej, amovel

    # 초기 위치 이동
    # JReady = [0, 0, 90, 0, 90, 0]
    # movej(JReady, vel=VELOCITY, acc=ACC)
    
    # 예시 1: TCP 기준 X축으로 30도 기울이기 (옆으로 갸웃)
    # [x, y, z, rx, ry, rz] -> 이동은 0, rx만 30도

    # movel(posx([401.822, 87.677, 173.491, 46.433, -178.642, 54.572]), vel=VELOCITY, acc=ACC, ref=DR_BASE) # 특정 좌표로 이동

    # 예시 2: TCP 기준 Y축으로 30도 기울이기 (앞뒤 끄덕)
    print("Tilting forward/backward (Pitch)")
    tilt_forward = posx([0, 0, 0, 0, 22, 0])
    movel(tilt_forward, vel=VELOCITY, acc=ACC, ref=DR_TOOL) 

    # print("Tilting sideways (Roll)")
    # tilt_sideways = posx([0, 0, 0, 20, 0, 0]) 
    # amovel(tilt_sideways, vel=VELOCITY, acc=ACC, ref=DR_TOOL) # ref=DR_TOOL이 핵심
    wait(1.0)
    movel(posx([452.286, 91.943, 191.094, 61.636, -177.07, 85.74]), vel=VELOCITY, acc=ACC, ref=DR_BASE) # 특정 좌표로 이동

    

    # wait(1.0)


def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 동작 수행"""
    rclpy.init(args=args)
    node = rclpy.create_node("move_basic", namespace=ROBOT_ID)

    # DR_init에 노드 설정
    DR_init.__dsr__node = node

    try:
        # 초기화는 한 번만 수행
        initialize_robot()

        # 작업 수행 (한 번만 호출)
        perform_task()

    except KeyboardInterrupt:
        print("\nNode interrupted by user. Shutting down...")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()