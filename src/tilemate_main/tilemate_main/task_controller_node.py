#!/usr/bin/env python3
# cobot1/task_controller_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int32

class TaskControllerNode(Node):
    """
    /robot/command(String) -> 내부 이벤트 토픽 발행
    - start: pause=false, stop_soft=false, run_once 1회 트리거(원하면)
    - once: run_once 1회 트리거
    - pause/resume: pause 토픽
    - stop/reset: stop_soft 토픽 (하드정지는 CommandNode가 별도로 처리)
    """
    def __init__(self):
        super().__init__("task_controller_node")

        self._run_token = 0

        self.pub_run = self.create_publisher(Int32, "/task/run_once", 10)
        self.pub_pause = self.create_publisher(Bool, "/task/pause", 10)
        self.pub_stop_soft = self.create_publisher(Bool, "/task/stop_soft", 10)

        self.create_subscription(String, "/robot/command", self._cb_cmd, 10)
        self.get_logger().info("TaskControllerNode ready: sub /robot/command")

    def _publish_pause(self, v: bool):
        m = Bool()
        m.data = bool(v)
        self.pub_pause.publish(m)

    def _publish_stop_soft(self, v: bool):
        m = Bool()
        m.data = bool(v)
        self.pub_stop_soft.publish(m)

    def _publish_run_once(self):
        self._run_token += 1
        m = Int32()
        m.data = self._run_token
        self.pub_run.publish(m)
        self.get_logger().info(f"[TASK] trigger run_once token={m.data}")

    def _cb_cmd(self, msg: String):
        cmd = msg.data.strip().lower()

        if cmd == "start":
            self.get_logger().info("[CMD] start")
            self._publish_stop_soft(False)
            self._publish_pause(False)
            # start를 “1회 실행”으로 보고 싶으면 아래 켜기
            # self._publish_run_once()

        elif cmd == "once":
            self.get_logger().info("[CMD] once")
            self._publish_stop_soft(False)
            self._publish_pause(False)
            self._publish_run_once()

        elif cmd == "pause":
            self.get_logger().warn("[CMD] pause")
            self._publish_pause(True)

        elif cmd == "resume":
            self.get_logger().info("[CMD] resume")
            self._publish_pause(False)

        elif cmd == "stop":
            self.get_logger().warn("[CMD] stop (soft)")
            self._publish_pause(False)
            self._publish_stop_soft(True)

        elif cmd == "reset":
            self.get_logger().warn("[CMD] reset")
            self._publish_pause(False)
            self._publish_stop_soft(True)

        else:
            self.get_logger().warn(f"[CMD] unknown: {cmd}")

def main(args=None):
    rclpy.init(args=args)
    node = TaskControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
