#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from tilemate_msgs.srv import Inspect


class InspectTilesService(Node):

    def __init__(self):
        super().__init__("inspect_tiles_service")

        self.cb_group = ReentrantCallbackGroup()

        self.srv = self.create_service(
            Inspect,
            "tile/inspect",
            self.inspect_callback,
            callback_group=self.cb_group,
        )

        self.get_logger().info("[INSPECT] service ready")

    def inspect_callback(self, request, response):

        del request

        self.get_logger().info("[INSPECT] start inspection")

        # 예시 anomaly score
        scores = [0.05, 0.12, 0.03]

        response.success = True
        response.message = "inspect_complete"
        response.anomaly_scores = scores

        return response


def main(args=None):

    rclpy.init(args=args)

    node = InspectTilesService()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()