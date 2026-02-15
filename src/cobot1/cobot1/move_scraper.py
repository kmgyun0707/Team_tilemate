#!/usr/bin/env python3
import time
import rclpy
import DR_init

from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# =========================
# 로봇 설정 상수
# =========================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

VELOCITY = 40
ACC = 60

ON, OFF = 1, 0

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def initialize_robot():
    """로봇의 Tool과 TCP를 설정"""
    from DSR_ROBOT2 import (
        set_tool, set_tcp, get_tool, get_tcp,
        ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS,
        get_robot_mode, set_robot_mode
    )

    # Tool/TCP 설정은 Manual에서 수행
    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)

    # 다시 Autonomous로 복귀
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(1.0)  # 설정 안정화

    print("#" * 50)
    print("Initializing robot with the following settings:")
    print(f"ROBOT_ID: {ROBOT_ID}")
    print(f"ROBOT_MODEL: {ROBOT_MODEL}")
    print(f"ROBOT_TCP: {get_tcp()}")
    print(f"ROBOT_TOOL: {get_tool()}")
    print(f"ROBOT_MODE (0:수동, 1:자동): {get_robot_mode()}")
    print(f"VELOCITY: {VELOCITY}")
    print(f"ACC: {ACC}")
    print("#" * 50)


def perform_task(node: Node):
    """로봇이 수행할 작업 + 그리퍼 퍼블리시"""
    from DSR_ROBOT2 import posx,movej,movel, posj # movel은 지금 주석처리라 posx만
    print("Performing grip task...")
    from DSR_ROBOT2 import (
        set_digital_output,
        get_digital_input,
        movej,wait
    )
    # 디지털 입력 신호 대기 함수
    def wait_digital_input(sig_num):
        while not get_digital_input(sig_num):
            wait(0.5)
            # print("Waiting for digital input...")


    # Release 동작
    def release():
        print("Releasing...")
        set_digital_output(2, ON)
        set_digital_output(1, OFF)

    # Grip 동작
    def grip():
        print("Gripping...")
        # release()
        set_digital_output(1, ON)
        set_digital_output(2, OFF)

    # ✅ OnRobot 그리퍼 명령 퍼블리셔
    gripper_pub = node.create_publisher(
        Float64MultiArray,
        "/onrobot/finger_width_controller/commands",
        10
    )

    def set_gripper(width_m: float):
        """
        width_m: finger width (m)
        예) 0.05 => 50mm
        """
        msg = Float64MultiArray()
        msg.data = [float(width_m)]  # unit : meter
        gripper_pub.publish(msg)
        node.get_logger().info(f"[GRIPPER] publish: {msg.data}")

    def pick_scraper():
        #밀대 파지전 위치 이동 
        pre_grasp_pos = posx([634.6129150390625, 70.18247985839844, 216.4523162841797, 52.00111389160156, 179.09434509277344, 52.347633361816406])
        movel(pre_grasp_pos,vel=60,acc=60)
        time.sleep(1.0)
        #밀대 파지위치 이동
        grasp_pos = posx([635.6554565429688, 69.99335479736328, 156.9518280029297, 122.97427368164062, 179.7987518310547, 123.16593170166016])
        movel(grasp_pos,vel=60,acc=60)
        time.sleep(1.0)
        set_gripper(0.003)
        time.sleep(4.0)

    def place_scraper():
        #밀대 파지전 위치 이동 
        pre_grasp_pos = posx([634.6129150390625, 70.18247985839844, 216.4523162841797, 52.00111389160156, 179.09434509277344, 52.347633361816406])
        movel(pre_grasp_pos,vel=60,acc=60)
        time.sleep(1.0)
        #밀대 파지위치 이동
        grasp_pos = posx([635.6554565429688, 69.99335479736328, 186.9518280029297, 122.97427368164062, 179.7987518310547, 123.16593170166016])
        movel(grasp_pos,vel=60,acc=60)
        time.sleep(1.0)
        set_gripper(0.04)
        time.sleep(4.0)



    # (예시) 초기 위치 및 목표 위치
    JReady = [0, 0, 90, 0, 90, 0]
    pos1 = posx([500, 80, 200, 150, 179, 150]) 

    node.get_logger().info(f"JReady = {JReady}")
    node.get_logger().info(f"pos1   = {pos1}")

    # 반복 동작
    while rclpy.ok():
        # # 얘는 접착제 
        # movej(JReady, vel=VELOCITY, acc=ACC)
        # set_gripper(0.060)
        # time.sleep(2.0)

        # pos2 = posx([374.6396179199219, -245.0562744140625, 278.8721008300781, 91.11102294921875, -139.75392150878906, 90.13805389404297])
        # movel(pos2,vel=60,acc=60)
        # time.sleep(2.0)

        # pos3 = posx([375.3787841796875, -288.3359680175781, 229.7924346923828, 91.1861801147461, -139.70118713378906, 90.18544006347656])
        # movel(pos3,vel=60,acc=60)
        # time.sleep(2.0)

        # set_gripper(0.034)
        # time.sleep(2.0)

        # pos4 = posx([341.48187255859375, -452.4921569824219, 362.4076843261719, 91.81242370605469, -139.04881286621094, 90.89047241210938])
        # movel(pos4,vel=60,acc=60)
        # time.sleep(1.0)

        # pos5 = posx([337.8330993652344, -118.81729125976562, 354.3833923339844, 94.10023498535156, -176.3974609375, 93.70670318603516])
        # movel(pos5,vel=60,acc=60)
        # time.sleep(1.0)
        
        # pos6 = posx([361.9256591796875, -82.01500701904297, 327.6671447753906, 126.4363784790039, -178.03363037109375, 128.0858917236328])
        # movel(pos6,vel=60,acc=60)
        # time.sleep(1.0)

        # pos7 = posx([411.6914978027344, 86.2454605102539, 200.359130859375, 93.36572265625, -141.7025146484375, 92.81163024902344])
        # movel(pos7,vel=60,acc=60)
        # time.sleep(1.0)

        # set_gripper(0.00)
        # time.sleep(12.0)

        movej(JReady, vel=VELOCITY, acc=ACC)
        #release()
        set_gripper(0.06)
        time.sleep(2.0)
        
        pick_scraper()

        #밀대 들고 위로 이동 
        pos3 = posx([634.6129150390625, 70.18247985839844, 216.4523162841797, 52.00111389160156, 179.09434509277344, 52.347633361816406])
        movel(pos3,vel=60,acc=60)
        time.sleep(1.0)

        # set_gripper(0.034)
        # time.sleep(2.0)
        #밀대 들고 가운데로 이동 
        pos4 = posx([480.86981201171875, 68.99758911132812, 167.26080322265625, 59.91958999633789, 179.1564178466797, 60.55112075805664])
        movel(pos4,vel=60,acc=60)
        time.sleep(1.0)

        pos5 = posj([6.65,16.37,75.84,0.56,87.29,95.63])
        movej(pos5, vel=40,acc=40)
        time.sleep(1.0)

        ########################################
    
        print("시멘트 펴바르기 작업 시작")
        tilt_right = posx([465.40,-34.13,160.68,92.35,-142.05,179.91])   #tilt +x
        movel(tilt_right, vel=40,acc=40)
        time.sleep(1.0)

        pos7 = posx([465.40,250.13,160.68,92.35,-142.05,179.91])
        movel(pos7, vel=40,acc=40)
        time.sleep(1.0)

        tilt_left = posx([462.86,274.11,160.68,93.23,148.62,-179.15]) # tilt -x
        movel(tilt_left, vel=20,acc=20)
        time.sleep(1.0)

        pos9 = posx([528.40,-34.13,160.68,93.23,148.62,-179.15])
        movel(pos9, vel=40,acc=40)
        time.sleep(1.0)

        # pos9 = posx([528.40,-34.13,160.68,92.35,-142.05,179.91])
        # movel(pos9, vel=40,acc=40)
        # time.sleep(1.0)


        # pos9 = posx([528.40,250.13,160.68,92.35,-142.05,179.91])
        # movel(pos9, vel=40,acc=40)
        time.sleep(1.0)
        print("시멘트 펴바르기 작업 종료")
        #########################################
        movej(JReady, vel=VELOCITY, acc=ACC)
        time.sleep(2.0)

        place_scraper()



def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node("move_scraper", namespace=ROBOT_ID)

    # DR_init에 노드 설정
    DR_init.__dsr__node = node

    try:
        initialize_robot()
        perform_task(node)

    except KeyboardInterrupt:
        print("\nInterrupted by user. Shutting down...")

    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        raise

    finally:
        try:
            node.destroy_node()
        except Exception:
            pass

        # init 안 된 상태에서 shutdown 방지용 (안전)
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
