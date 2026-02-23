#!/usr/bin/env python3
# cobot1/command_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from dsr_msgs2.srv import MoveStop

from tilemate_main.robot_config import RobotConfig

class InterruptNode(Node):
    """
    /robot/command 수신해서,
    stop/pause/reset에 대해 MoveStop 서비스를 호출(하드 정지/홀드).
    """
    def __init__(self, cfg: RobotConfig):
        super().__init__("command_node")

        self.cfg = cfg
        self.stop_srv_name = f"/{cfg.robot_id}/motion/move_stop"

        self.stop_cli = self.create_client(MoveStop, self.stop_srv_name)
        while not self.stop_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f"Waiting for MoveStop: {self.stop_srv_name}")

        self.create_subscription(String, "/robot/command", self._cb_cmd, 10)
        self.get_logger().info("CommandNode ready (hard stop): sub /robot/command")

    def _call_move_stop(self, stop_mode: int):
        req = MoveStop.Request()
        req.stop_mode = int(stop_mode)
        self.stop_cli.call_async(req)

    def _cb_cmd(self, msg: String):
        cmd = msg.data.strip().lower()

        if cmd == "stop":
            self.get_logger().warn("[CMD] stop -> MoveStop(QSTOP=1)")
            self._call_move_stop(1)

        elif cmd == "pause":
            self.get_logger().warn("[CMD] pause -> MoveStop(HOLD=3)")
            self._call_move_stop(3)

        elif cmd == "reset":
            self.get_logger().warn("[CMD] reset -> MoveStop(QSTOP=1)")
            self._call_move_stop(1)

        # start/resume/once는 TaskController가 처리, 여기서는 무시
        else:
            return

def main(args=None):
    rclpy.init(args=args)
    cfg = RobotConfig()
    node = InterruptNode(cfg)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
