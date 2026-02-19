#!/usr/bin/env python3
import time
import threading

import rclpy
import DR_init

from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from std_msgs.msg import Float64MultiArray, Int32, String
from dsr_msgs2.srv import MoveStop

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

STOP_SERVICE_NAME = f"/{ROBOT_ID}/motion/move_stop"

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# =========================
# 전역 상태 플래그
# =========================
start_flag = True   # start gating 쓰려면 False로 두고 start 받으면 True
stop_now_flag = False
pause_flag = False


class CommandNode(Node):
    """
    comm_node: /robot/command 구독 + MoveStop 호출 전용
    - ★ 전용 Executor로 spin (rclpy.spin 사용 금지)
    - ★ DSR_ROBOT2 함수 절대 호출 금지
    """
    def __init__(self):
        super().__init__("move_scraper_comm")

        self.create_subscription(String, "/robot/command", self.trigger_cb, 10)

        self.stop_cli = self.create_client(MoveStop, STOP_SERVICE_NAME)
        while not self.stop_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f"Waiting for MoveStop service: {STOP_SERVICE_NAME}")

        self.get_logger().info("CommandNode ready. /robot/command: start|stop|pause|resume")

    def trigger_cb(self, msg: String):
        global start_flag, stop_now_flag, pause_flag

        cmd = msg.data.strip().lower()

        if cmd == "start":
            self.get_logger().info("[CMD] start")
            start_flag = True
            stop_now_flag = False
            pause_flag = False

        elif cmd == "stop":
            self.get_logger().warn("[CMD] stop -> MoveStop(QSTOP=1)")
            stop_now_flag = True
            pause_flag = False

            req = MoveStop.Request()
            req.stop_mode = 1  # DR_QSTOP
            self.stop_cli.call_async(req)

        elif cmd == "pause":
            self.get_logger().warn("[CMD] pause -> MoveStop(HOLD=3)")
            pause_flag = True

            req = MoveStop.Request()
            req.stop_mode = 3  # DR_HOLD
            self.stop_cli.call_async(req)

        elif cmd == "resume":
            self.get_logger().info("[CMD] resume")
            pause_flag = False

        elif cmd == "reset":
            self.get_logger().warn("[CMD] reset -> stop flags reset + MoveStop(QSTOP=1)")
            start_flag = True
            stop_now_flag = True
            pause_flag = False

            req = MoveStop.Request()
            req.stop_mode = 1
            self.stop_cli.call_async(req)

        else:
            self.get_logger().warn(f"[CMD] unknown: {cmd}")


def _spin_comm_node(comm_node: Node, stop_event: threading.Event):
    """
    comm_node를 전용 executor로 spin.
    - 글로벌 executor 사용 금지 (DSR_ROBOT2의 spin_until_future_complete와 충돌 방지)
    """
    exec_ = SingleThreadedExecutor()
    exec_.add_node(comm_node)
    try:
        while rclpy.ok() and not stop_event.is_set():
            exec_.spin_once(timeout_sec=0.1)
    finally:
        try:
            exec_.remove_node(comm_node)
        except Exception:
            pass


def initialize_robot():
    """로봇의 Tool과 TCP를 설정 (motion_node로만 수행)"""
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
    time.sleep(1.0)

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


def perform_task(motion_node: Node):
    """
    motion_node: DSR_ROBOT2 motion 전용
    - ★ rclpy.spin / executor.spin 호출 금지
    - stop/pause는 comm_node가 MoveStop으로 즉시 끊음
    - 여기서는 stop_now_flag/pause_flag로 종료/대기만 수행
    """
    from DSR_ROBOT2 import (
        posx, posj, movej, movel,
        set_digital_output, get_digital_input, wait
    )

    # 디지털 입력 신호 대기 함수
    def wait_digital_input(sig_num):
        while not get_digital_input(sig_num):
            wait(0.5)

    # Release 동작
    def release():
        motion_node.get_logger().info("Releasing...")
        set_digital_output(2, ON)
        set_digital_output(1, OFF)

    # Grip 동작
    def grip():
        motion_node.get_logger().info("Gripping...")
        set_digital_output(1, ON)
        set_digital_output(2, OFF)

    #  OnRobot 그리퍼 명령 퍼블리셔
    gripper_pub = motion_node.create_publisher(
        Float64MultiArray,
        "/onrobot/finger_width_controller/commands",
        10
    )

    step_pub = motion_node.create_publisher(Int32, "/robot/step", 10)
    state_pub = motion_node.create_publisher(String, "/robot/state", 10)

    def set_gripper(width_m: float):
        """
        width_m: finger width (m)
        예) 0.05 => 50mm
        """
        msg = Float64MultiArray()
        msg.data = [float(width_m)]  # unit : meter
        gripper_pub.publish(msg)
        motion_node.get_logger().info(f"[GRIPPER] publish: {msg.data}")

    def publish_step(step: int, state: str):
        step_msg = Int32()
        step_msg.data = int(step)
        step_pub.publish(step_msg)

        state_msg = String()
        state_msg.data = state
        state_pub.publish(state_msg)

    def wait_if_paused():
        global pause_flag, stop_now_flag
        while rclpy.ok() and pause_flag and not stop_now_flag:
            time.sleep(0.05)

    # -------------------------
    # 작업 시퀀스 정의
    # -------------------------
    def pick_scraper():
        publish_step(1, "접착제 도포준비중")

        # 밀대 파지전 위치 이동
        pre_grasp_pos = posx([
            634.6129150390625, 70.18247985839844, 216.4523162841797,
            52.00111389160156, 179.09434509277344, 52.347633361816406
        ])
        movel(pre_grasp_pos, vel=60, acc=60)
        time.sleep(0.2)
        wait_if_paused()

        # 밀대 파지 위치 이동
        grasp_pos = posx([
            635.6554565429688, 69.99335479736328, 156.9518280029297,
            122.97427368164062, 179.7987518310547, 123.16593170166016
        ])
        movel(grasp_pos, vel=60, acc=60)
        time.sleep(0.2)
        wait_if_paused()

        # 그리퍼 닫기(예: 3mm)
        set_gripper(0.003)
        time.sleep(0.2)
        wait_if_paused()

    def place_scraper():
        publish_step(3, "타일파지 준비중")

        # 밀대 파지전 위치 이동
        pre_grasp_pos = posx([
            634.6129150390625, 70.18247985839844, 216.4523162841797,
            52.00111389160156, 179.09434509277344, 52.347633361816406
        ])
        movel(pre_grasp_pos, vel=60, acc=60)
        time.sleep(0.2)
        wait_if_paused()

        # 밀대 내려놓는 위치
        grasp_pos = posx([
            635.6554565429688, 69.99335479736328, 186.9518280029297,
            122.97427368164062, 179.7987518310547, 123.16593170166016
        ])
        movel(grasp_pos, vel=60, acc=60)
        time.sleep(0.2)
        wait_if_paused()

        # 그리퍼 열기(예: 40mm)
        set_gripper(0.04)
        time.sleep(0.2)
        wait_if_paused()

    # -------------------------
    # 초기 위치
    # -------------------------
    JReady = [0, 0, 90, 0, 90, 0]
    motion_node.get_logger().info(f"JReady = {JReady}")

    global start_flag, stop_now_flag

    # start gating 쓰려면:
    # while rclpy.ok() and not start_flag:
    #     time.sleep(0.05)

    # -------------------------
    # 반복 동작
    # -------------------------
    while rclpy.ok():
        if stop_now_flag:
            motion_node.get_logger().warn("STOP flag set -> exiting perform_task() loop")
            break

        wait_if_paused()

        movej(JReady, vel=VELOCITY, acc=ACC)
        set_gripper(0.06)  # release
        time.sleep(0.2)
        wait_if_paused()

        pick_scraper()

        # 밀대 들고 위로 이동
        pos3 = posx([
            634.6129150390625, 70.18247985839844, 216.4523162841797,
            52.00111389160156, 179.09434509277344, 52.347633361816406
        ])
        movel(pos3, vel=60, acc=60)
        time.sleep(0.2)
        wait_if_paused()

        # 밀대 들고 가운데로 이동
        pos4 = posx([
            480.86981201171875, 68.99758911132812, 167.26080322265625,
            59.91958999633789, 179.1564178466797, 60.55112075805664
        ])
        movel(pos4, vel=60, acc=60)
        time.sleep(0.2)
        wait_if_paused()

        pos5 = posj([6.65, 16.37, 75.84, 0.56, 87.29, 95.63])
        movej(pos5, vel=40, acc=40)
        time.sleep(0.2)
        wait_if_paused()

        # -------------------------
        # 시멘트 펴바르기 작업
        # -------------------------
        motion_node.get_logger().info("시멘트 펴바르기 작업 시작")
        publish_step(2, "접착제 도포중")

        tilt_right = posx([465.40, -34.13, 160.68, 92.35, -142.05, 179.91])
        movel(tilt_right, vel=40, acc=40)
        time.sleep(0.2)
        wait_if_paused()

        pos7 = posx([485.40, 250.13, 160.68, 92.35, -142.05, 179.91])
        movel(pos7, vel=40, acc=40)
        time.sleep(0.2)
        wait_if_paused()

        tilt_left = posx([485.40, 274.11, 160.68, 93.23, 148.62, -179.15])
        movel(tilt_left, vel=40, acc=40)
        time.sleep(0.2)
        wait_if_paused()

        pos8 = posx([505.40, -115.13, 160.68, 93.23, 148.62, -179.15])
        movel(pos8, vel=40, acc=40)
        time.sleep(0.2)
        wait_if_paused()

        tilt_right2 = posx([505.40, -115.13, 165.68, 92.35, -142.05, 179.91])
        movel(tilt_right2, vel=40, acc=40)
        time.sleep(0.2)
        wait_if_paused()

        tilt_right2b = posx([505.40, -115.13, 160.68, 92.35, -142.05, 179.91])
        movel(tilt_right2b, vel=40, acc=40)
        time.sleep(0.2)
        wait_if_paused()

        pos9 = posx([505.40, 250.13, 165.68, 92.35, -142.05, 179.91])
        movel(pos9, vel=40, acc=40)
        time.sleep(0.2)
        wait_if_paused()

        motion_node.get_logger().info("시멘트 펴바르기 작업 종료")

        # 홈 복귀
        movej(JReady, vel=VELOCITY, acc=ACC)
        time.sleep(0.2)
        wait_if_paused()

        place_scraper()


def main(args=None):
    rclpy.init(args=args)

    # 1) comm_node 생성 + 전용 executor thread로 spin
    comm_node = CommandNode()
    stop_event = threading.Event()
    comm_thread = threading.Thread(target=_spin_comm_node, args=(comm_node, stop_event), daemon=True)
    comm_thread.start()

    # 2) motion_node 생성(DSR 전용) + DR_init에 등록
    motion_node = rclpy.create_node("move_scraper", namespace=ROBOT_ID)
    DR_init.__dsr__node = motion_node  # DSR_ROBOT2는 이 노드로만 동작

    try:
        initialize_robot()
        motion_node.get_logger().info("Ready. /robot/command: start | stop | pause | resume | reset")
        perform_task(motion_node)

    except KeyboardInterrupt:
        motion_node.get_logger().warn("Interrupted by user. Shutting down...")

    finally:
        # comm thread 종료 신호
        stop_event.set()
        time.sleep(0.2)

        try:
            motion_node.destroy_node()
        except Exception:
            pass

        try:
            comm_node.destroy_node()
        except Exception:
            pass

        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
