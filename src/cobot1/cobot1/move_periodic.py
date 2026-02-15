import time
import rclpy
import DR_init

# 로봇 설정 상수
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA"

# 이동 속도 및 가속도
VELOCITY = 60
ACC = 60

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def initialize_robot():
    """로봇의 Tool과 TCP를 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp  # 필요한 기능만 임포트

    # 설정된 상수 출력
    print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    print("#" * 50)
    print("Initializing robot with the following settings:")
    print(f"ROBOT_ID: {ROBOT_ID}")
    print(f"ROBOT_MODEL: {ROBOT_MODEL}")
    print(f"ROBOT_TCP: {ROBOT_TCP}")
    print(f"ROBOT_TOOL: {ROBOT_TOOL}")
    print(f"VELOCITY: {VELOCITY}")
    print(f"ACC: {ACC}")
    print("#" * 50)

    # Tool과 TCP 설정
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)


'''
판떼기 그립자세
TCP pose: ([635.6554565429688, 69.99335479736328, 156.9518280029297, 122.97427368164062, 179.7987518310547, 123.16593170166016], 2)
판떼기 잡고 올리기 
TCP pose: ([634.6129150390625, 70.18247985839844, 216.4523162841797, 52.00111389160156, 179.09434509277344, 52.347633361816406], 2)
이동
TCP pose: ([480.86981201171875, 68.99758911132812, 167.26080322265625, 59.91958999633789, 179.1564178466797, 60.55112075805664], 2)
회자ㅓㄴ
TCP pose: ([480.86981201171875, 68.99758911132812, 167.26080322265625, 59.91958999633789, 179.1564178466797, 60.55112075805664], 2)
TCP pose: ([463.34375, 142.89772033691406, 219.06289672851562, 87.79329681396484, 146.30458068847656, 177.22280883789062], 2)
TCP pose: ([460.2585754394531, 183.42869567871094, 160.65672302246094, 88.42152404785156, 146.27500915527344, 177.89328002929688], 2)
TCP pose: ([460.0016174316406, -68.7561264038086, 152.9974365234375, 87.34696960449219, 137.4104766845703, 176.76553344726562], 2)
TCP pose: ([480.70404052734375, -24.42903709411621, 152.40298461914062, 88.0481948852539, -134.5972900390625, 177.98358154296875], 2)

반대각도 

'''



def perform_task():
    # from DSR_ROBOT2 import move_periodic,DR_BASE,move_spiral,DR_AXIS_Z
    from DSR_ROBOT2 import get_current_posx,DR_BASE
    while True:

        pos = get_current_posx(ref=DR_BASE)
        print("TCP pose:", pos)
        time.sleep(1.0)
# #     # spiral_radius = 30.0 # 3cm 반경
# #     # v_list = [60.0, 60.0]   
# #     # a_list = [100.0, 100.0]
# #     '''
# #     (function) def move_spiral(
# #     rev: int = 10,
# #     rmax: int = 10,
# #     lmax: int = 0,
# #     vel: Any | None = None,
# #     acc: Any | None = None,
# #     time: Any | None = None,
# #     axis: int = DR_AXIS_Z,
# #     ref: int = DR_TOOL,
# #     v: Any | None = None,
# #     a: Any | None = None,
# #     t: Any | None = None
# # ) -> Any
# #     '''
#     print("pass1111111")
#     move_spiral(rev=5, rmax=20, v=60, a=100)
#     print("pass2222222")
#     # move_spiral(rev=5, rmax=30,lmax=0,t=15.0,axis=DR_AXIS_Z,ref=DR_BASE)
#     #move_spiral(rev=5, rmax=spiral_radius, lmax=0,t=15.0,axis=DR_AXIS_Z, ref=DR_BASE)
#     #move_periodic(amp=[100,100,100,0,0,0],period=[3.2,1.6,0,0,0,0],atime=3.1,repeat=2,ref=DR_BASE)
    


def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 동작 수행"""
    rclpy.init(args=args)
    node = rclpy.create_node("move_periodic", namespace=ROBOT_ID)

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