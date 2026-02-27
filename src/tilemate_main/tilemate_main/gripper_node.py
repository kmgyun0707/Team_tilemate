#!/usr/bin/env python3
# cobot1/gripper_node.py
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Float64MultiArray

class GripperNode(Node):
    """
    /gripper/width_m (Float64)  ->  /onrobot/finger_width_controller/commands (Float64MultiArray)
    """
    def __init__(self):
        super().__init__("gripper_node")

        self._out_pub = self.create_publisher(
            Float64MultiArray,
            "/onrobot/finger_width_controller/commands",
            10
        )

        self.create_subscription(Float64, "/gripper/width_m", self._cb_width, 10)
        self.get_logger().info("GripperNode ready: sub /gripper/width_m -> pub /onrobot/.../commands")

    def _cb_width(self, msg: Float64):
        out = Float64MultiArray()
        out.data = [float(msg.data)]
        self._out_pub.publish(out)
        self.get_logger().info(f"[GRIPPER] width_m={out.data[0]:.4f}")

def main(args=None):
    rclpy.init(args=args)
    node = GripperNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
