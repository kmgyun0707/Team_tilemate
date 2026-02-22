# 타일 적용 테스트 코드
# 1. TCP 기준으로 기울이기 (Pitch, Roll)
# 모듈화(함수) 완료 

import rclpy
import DR_init
import time

# 로봇 설정 상수 (필요에 따라 수정)
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

# 이동 속도 및 가속도 (필요에 따라 수정)
VELOCITY = 30
ACC = 30
# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def initialize_robot():
    """로봇의 Tool과 TCP를 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp,get_tool,get_tcp,ROBOT_MODE_MANUAL,ROBOT_MODE_AUTONOMOUS  # 필요한 기능만 임포트
    from DSR_ROBOT2 import get_robot_mode,set_robot_mode

    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
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

# 타일 떼는 함수(기울려서 떼는 방식)
def detach_tile():
    """타일을 기울여서 떼는 작업"""
    print("Detaching tile by tilting...")
    # DR_TOOL 추가 Import 필수
    from DSR_ROBOT2 import posx, movel,wait, DR_TOOL,add_tcp,get_tcp,set_tcp,DR_BASE,DR_WORLD,amovej, amovel,set_robot_mode,ROBOT_MODE_MANUAL,ROBOT_MODE_AUTONOMOUS
     
    # TCP 오프셋 추가
    set_robot_mode(ROBOT_MODE_MANUAL)     # TCP 설정을 위해 수동 모드로 전환
    tcp_name = "MySuction_v1"
    tcp_offset = [0, 0, 265, 0, 0, 0] # x, y, z, a, b, c
    add_tcp(tcp_name, tcp_offset)# 이 함수도 TCP 설정을 위해 수동 모드에서 호출해야 함
    set_tcp(tcp_name)            # TCP 이름 설정
    set_robot_mode(ROBOT_MODE_AUTONOMOUS) # 자동 모드로 전환하여 TCP 적용
    
    # 잠시 대기
    wait(0.5)
    print(f"Current TCP: {get_tcp()}")
    tilt_forward = posx([0, 0, 0, 0, 22, 0]) # 22도는 되어야 기울었을때 타일이 떨어짐
    movel(tilt_forward, vel=1, acc=1, ref=DR_TOOL,time=0.7) # 

def perform_task():
    """로봇이 수행할 작업"""
    print("Performing task...")
    # DR_TOOL 추가 Import 필수
    from DSR_ROBOT2 import posx, movel, wait,DR_BASE

    # 특정 좌표로 이동 (DR_BASE 참조)
    movel(posx([401.822, 87.677, 173.491, 46.433, -178.642, 54.572]), vel=VELOCITY, acc=ACC, ref=DR_BASE)
    detach_tile() # 타일 떼는 작업 수행
    wait(1.0)
    movel(posx([452.286, 91.943, 200.094, 46.433, -178.642, 54.572]), vel=VELOCITY, acc=ACC, ref=DR_BASE)

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