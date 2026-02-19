# cobot1/scraper_task.py
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

import DR_init

ON, OFF = 1, 0


@dataclass
class RobotConfig:
    robot_id: str = "dsr01"
    robot_model: str = "m0609"
    tool: str = "Tool Weight"
    tcp: str = "GripperDA_v1"
    vel: float = 40
    acc: float = 60


class GripperController:
    """OnRobot finger_width_controller publisher wrapper (DI 대상)."""
    def __init__(self, node: Node, topic: str = "/onrobot/finger_width_controller/commands"):
        self._node = node
        self._pub = node.create_publisher(Float64MultiArray, topic, 10)

    def set_width(self, width_m: float):
        msg = Float64MultiArray()
        msg.data = [float(width_m)]
        self._pub.publish(msg)
        self._node.get_logger().info(f"[GRIPPER] publish width(m): {msg.data[0]:.4f}")


class ScraperTask:
    """
    스크래퍼(밀대) 작업 시퀀스.
    - node, gripper는 외부에서 주입받음
    - Firebase/상태머신은 몰라도 됨(순수 동작 로직)
    """
    def __init__(self, node: Node, gripper: GripperController, cfg: RobotConfig):
        self.node = node
        self.gripper = gripper
        self.cfg = cfg

    def initialize_robot(self):
        """Tool/TCP 설정만 담당. (메인에서도 한 번만 호출 가능)"""
        from DSR_ROBOT2 import (
            set_tool, set_tcp, get_tool, get_tcp,
            ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS,
            get_robot_mode, set_robot_mode
        )

        set_robot_mode(ROBOT_MODE_MANUAL)
        set_tool(self.cfg.tool)
        set_tcp(self.cfg.tcp)
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)

        time.sleep(1.0)
        print("#" * 50)
        print("ScraperTask robot init:")
        print(f"ROBOT_ID: {self.cfg.robot_id}")
        print(f"ROBOT_MODEL: {self.cfg.robot_model}")
        print(f"ROBOT_TCP: {get_tcp()}")
        print(f"ROBOT_TOOL: {get_tool()}")
        print(f"ROBOT_MODE (0:수동, 1:자동): {get_robot_mode()}")
        print("#" * 50)

    # -------------------------
    # 내부 동작 프리미티브
    # -------------------------
    def _pick_scraper(self):
        from DSR_ROBOT2 import posx, movel

        pre = posx([634.6129, 70.1825, 216.4523, 52.0011, 179.0943, 52.3476])
        movel(pre, vel=60, acc=60)
        time.sleep(1.0)

        grasp = posx([635.6555, 69.9934, 156.9518, 122.9743, 179.7988, 123.1659])
        movel(grasp, vel=60, acc=60)
        time.sleep(1.0)

        # 집기: 예시 3mm
        self.gripper.set_width(0.003)
        time.sleep(4.0)

        # 들고 위로
        movel(pre, vel=60, acc=60)
        time.sleep(1.0)

    def _place_scraper(self):
        from DSR_ROBOT2 import posx, movel

        pre = posx([634.6129, 70.1825, 216.4523, 52.0011, 179.0943, 52.3476])
        movel(pre, vel=60, acc=60)
        time.sleep(1.0)

        place = posx([635.6555, 69.9934, 186.9518, 122.9743, 179.7988, 123.1659])
        movel(place, vel=60, acc=60)
        time.sleep(1.0)

        # 놓기: 예시 40mm
        self.gripper.set_width(0.040)
        time.sleep(4.0)

        movel(pre, vel=60, acc=60)
        time.sleep(1.0)

    def _spread_cement(self):
        """시멘트 펴바르기 구간만 분리."""
        from DSR_ROBOT2 import posx, movel

        print("시멘트 펴바르기 작업 시작")

        tilt_right = posx([465.40, -34.13, 160.68, 92.35, -142.05, 179.91])
        movel(tilt_right, vel=40, acc=40)
        time.sleep(1.0)

        pos7 = posx([465.40, 250.13, 160.68, 92.35, -142.05, 179.91])
        movel(pos7, vel=40, acc=40)
        time.sleep(1.0)

        tilt_left = posx([462.86, 274.11, 160.68, 93.23, 148.62, -179.15])
        movel(tilt_left, vel=20, acc=20)
        time.sleep(1.0)

        pos9 = posx([528.40, -34.13, 160.68, 93.23, 148.62, -179.15])
        movel(pos9, vel=40, acc=40)
        time.sleep(1.0)

        print("시멘트 펴바르기 작업 종료")

    # -------------------------
    # 외부에 노출되는 API (메인에서 호출)
    # -------------------------
    def run_once(self):
        """
        스크래퍼 1회 사이클.
        메인 루프에서 원하는 타이밍에 호출하면 됨.
        """
        from DSR_ROBOT2 import movej, movel, posx, posj

        JReady = [0, 0, 90, 0, 90, 0]

        movej(JReady, vel=self.cfg.vel, acc=self.cfg.acc)
        self.gripper.set_width(0.060)  # 초기 오픈
        time.sleep(2.0)

        self._pick_scraper()

        # 가운데로 이동
        mid = posx([480.8698, 68.9976, 167.2608, 59.9196, 179.1564, 60.5511])
        movel(mid, vel=60, acc=60)
        time.sleep(1.0)

        # 자세 변경
        p5 = posj([6.65, 16.37, 75.84, 0.56, 87.29, 95.63])
        movej(p5, vel=40, acc=40)
        time.sleep(1.0)

        self._spread_cement()

        movej(JReady, vel=self.cfg.vel, acc=self.cfg.acc)
        time.sleep(2.0)

        self._place_scraper()
