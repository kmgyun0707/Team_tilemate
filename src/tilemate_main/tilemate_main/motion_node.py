#!/usr/bin/env python3
# cobot1/motion_node.py
import time
import threading

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Int32, Float64

import DR_init
from cobot1.robot_config import RobotConfig
from cobot1.scraper_task_lib import ScraperTask

class _GripperClient:
    """motion_node에서 gripper_node로 폭 명령을 보내기 위한 thin client."""
    def __init__(self, node: Node):
        self._node = node
        self._pub = node.create_publisher(Float64, "/gripper/width_m", 10)

    def set_width(self, width_m: float):
        msg = Float64()
        msg.data = float(width_m)
        self._pub.publish(msg)
        self._node.get_logger().info(f"[GRIPPER->CMD] width_m={msg.data:.4f}")

class MotionNode(Node):
    """
    - DSR_ROBOT2 전용 노드
    - /task/run_once 증가 시 1사이클 실행
    - /task/pause True면 대기
    - /task/stop_soft True면 루프/작업 중단(논리 stop)
    """
    def __init__(self, cfg: RobotConfig):
        super().__init__("motion_node", namespace=cfg.robot_id)

        # DR_init 연결 (DSR_ROBOT2는 이 노드로만 동작)
        DR_init.__dsr__id = cfg.robot_id
        DR_init.__dsr__model = cfg.robot_model
        DR_init.__dsr__node = self

        self.cfg = cfg

        self._pause = False
        self._stop_soft = False

        self._run_token = 0
        self._run_event = threading.Event()
        self._lock = threading.Lock()

        self.create_subscription(Int32, "/task/run_once", self._cb_run_once, 10)
        self.create_subscription(Bool, "/task/pause", self._cb_pause, 10)
        self.create_subscription(Bool, "/task/stop_soft", self._cb_stop_soft, 10)

        self.gripper = _GripperClient(self)
        self.task = ScraperTask(self.gripper, cfg)

        # 워커 스레드
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        # 초기화는 노드 시작 시 1회
        self.task.initialize_robot()
        self.get_logger().info("MotionNode ready: wait /task/run_once")

    def _cb_run_once(self, msg: Int32):
        with self._lock:
            # 같은 값 반복 수신 방지(단순 안전장치)
            if msg.data <= self._run_token:
                return
            self._run_token = int(msg.data)
        self.get_logger().info(f"[TASK] run_once token={self._run_token}")
        self._run_event.set()

    def _cb_pause(self, msg: Bool):
        self._pause = bool(msg.data)
        self.get_logger().warn(f"[TASK] pause={self._pause}")

    def _cb_stop_soft(self, msg: Bool):
        self._stop_soft = bool(msg.data)
        self.get_logger().warn(f"[TASK] stop_soft={self._stop_soft}")

    def _wait_if_paused(self):
        while rclpy.ok() and self._pause and not self._stop_soft:
            time.sleep(0.05)

    def _worker_loop(self):
        while rclpy.ok():
            self._run_event.wait(timeout=0.1)
            if not self._run_event.is_set():
                continue
            self._run_event.clear()

            if self._stop_soft:
                self.get_logger().warn("[TASK] stop_soft is True -> skip run_once")
                continue

            # pause 대기
            self._wait_if_paused()
            if self._stop_soft:
                continue

            try:
                self.get_logger().info("[TASK] run_once start")
                self.task.run_once()
                self.get_logger().info("[TASK] run_once done")
            except Exception as e:
                self.get_logger().error(f"[TASK] run_once exception: {e}")

def main(args=None):
    rclpy.init(args=args)
    cfg = RobotConfig()
    node = MotionNode(cfg)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
