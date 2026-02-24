#!/usr/bin/env python3
# tilemate_main/interrupt_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from dsr_msgs2.srv import MoveStop


class InterruptNode(Node):
    def __init__(self):
        super().__init__("interrupt_node")

        # ✅ 이름 통일: stop_soft 퍼블리셔는 pub_stop_soft로 고정
        self.pub_pause = self.create_publisher(Bool, "/task/pause", 10)
        self.pub_stop_soft = self.create_publisher(Bool, "/task/stop_soft", 10)

        # Option B resume triggers
        self.pub_scraper_resume = self.create_publisher(Bool, "/scraper/resume", 10)
        self.pub_tile_resume = self.create_publisher(Bool, "/tile/resume", 10)

        self.create_subscription(String, "/robot/command", self._cb_cmd, 10)

        self.cli_move_stop = self.create_client(MoveStop, "/dsr01/motion/move_stop")

        self.get_logger().error("### INTERRUPT_NODE VERSION: OPTION_B_RESUME_NO_MOVERESUME ###")
        self.get_logger().info("InterruptNode ready: sub /robot/command")

    def _pub_bool(self, pub, v: bool, tag: str):
        m = Bool()
        m.data = bool(v)
        pub.publish(m)
        self.get_logger().warn(f"[PUB] {tag}={m.data}")

    def _call_move_stop(self, stop_mode: int = 1):
        if not self.cli_move_stop.service_is_ready():
            self.get_logger().warn("[MoveStop] service not ready yet, waiting...")
            if not self.cli_move_stop.wait_for_service(timeout_sec=1.0):
                self.get_logger().error("[MoveStop] service unavailable")
                return

        req = MoveStop.Request()
        req.stop_mode = int(stop_mode)

        self.get_logger().warn(f"[MoveStop] request send: stop_mode={stop_mode} (/dsr01/motion/move_stop)")
        fut = self.cli_move_stop.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)

        if fut.done() and fut.result() is not None:
            self.get_logger().warn(f"[MoveStop] response: success={fut.result().success}")
        else:
            self.get_logger().warn("[MoveStop] no response / timeout")

    def _do_stop(self):
        self._pub_bool(self.pub_stop_soft, True, "/task/stop_soft")
        self._call_move_stop(stop_mode=1)

    def _do_resume_option_b(self):
        # ✅ 1) stop_soft 먼저 해제
        self._pub_bool(self.pub_stop_soft, False, "/task/stop_soft (first)")

        # ✅ 2) pause 해제
        self._pub_bool(self.pub_pause, False, "/task/pause")

        # ✅ 3) 앱 레벨 resume 트리거
        self._pub_bool(self.pub_scraper_resume, True, "/scraper/resume")
        self._pub_bool(self.pub_tile_resume, True, "/tile/resume")

        # ❌ MoveResume 호출 안 함

    def _cb_cmd(self, msg: String):
        cmd = (msg.data or "").strip().lower()
        self.get_logger().warn(f"[CMD] recv='{cmd}'")

        if cmd in ("stop", "stop_soft", "halt"):
            self._do_stop()
        elif cmd in ("resume", "continue"):
            self._do_resume_option_b()
        elif cmd in ("pause",):
            self._pub_bool(self.pub_pause, True, "/task/pause")
        elif cmd in ("unpause", "play"):
            self._pub_bool(self.pub_pause, False, "/task/pause")
        else:
            self.get_logger().warn(f"[CMD] unknown: {cmd}")


def main(args=None):
    rclpy.init(args=args)
    node = InterruptNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()