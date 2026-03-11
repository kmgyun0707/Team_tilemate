#!/usr/bin/env python3
"""
firebase_bridge/firebase_bridge.py
메인 진입점: ROS2 노드 실행
"""

import rclpy
from firebase_admin import db

from .ros_node import FirebaseBridgeNode


def main(args=None):
    rclpy.init(args=args)
    node = FirebaseBridgeNode()
    print("Firebase Bridge running... (Ctrl+C to stop)")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        for cleanup in [
            lambda: db.reference("/robot_status").update({"state": "대기", "current_step": 0}),
            lambda: node.destroy_node(),
            lambda: rclpy.shutdown(),
        ]:
            try:
                cleanup()
            except Exception:
                pass


if __name__ == "__main__":
    main()
