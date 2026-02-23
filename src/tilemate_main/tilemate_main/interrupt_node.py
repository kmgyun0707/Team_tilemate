#!/usr/bin/env python3
# cobot1/command_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from dsr_msgs2.srv import MoveStop, MovePause, MoveResume
from tilemate_main.robot_config import RobotConfig


class InterruptNode(Node):
    """
    /robot/command 수신해서,
    stop/pause/resume/reset에 대해 Doosan motion 서비스를 호출.
    - pause  -> MovePause
    - resume -> MoveResume
    - stop/reset -> MoveStop(quick=1)
    """
    def __init__(self, cfg: RobotConfig):
        super().__init__("command_node")
        self.cfg = cfg

        self.srv_stop   = f"/{cfg.robot_id}/motion/move_stop"
        self.srv_pause  = f"/{cfg.robot_id}/motion/move_pause"
        self.srv_resume = f"/{cfg.robot_id}/motion/move_resume"

        self.cli_stop   = self.create_client(MoveStop,   self.srv_stop)
        self.cli_pause  = self.create_client(MovePause,  self.srv_pause)
        self.cli_resume = self.create_client(MoveResume, self.srv_resume)

        self._wait_srv(self.cli_stop,   self.srv_stop)
        self._wait_srv(self.cli_pause,  self.srv_pause)
        self._wait_srv(self.cli_resume, self.srv_resume)

        self.create_subscription(String, "/robot/command", self._cb_cmd, 10)
        self.get_logger().info(
            f"InterruptNode ready: sub /robot/command | stop={self.srv_stop} pause={self.srv_pause} resume={self.srv_resume}"
        )

    def _wait_srv(self, cli, name: str):
        while not cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f"Waiting for service: {name}")

    def _log_future(self, tag: str, fut):
        try:
            res = fut.result()
            self.get_logger().warn(f"[{tag}] response: success={getattr(res, 'success', None)}")
        except Exception as e:
            self.get_logger().error(f"[{tag}] call failed: {e}")

    def _call_move_stop(self, stop_mode: int):
        # stop_mode: 1=quick stop, 2=slow stop (Doosan spec)
        req = MoveStop.Request()
        req.stop_mode = int(stop_mode)
        self.get_logger().warn(f"[MoveStop] request send: stop_mode={req.stop_mode} ({self.srv_stop})")
        fut = self.cli_stop.call_async(req)
        fut.add_done_callback(lambda f: self._log_future("MoveStop", f))

    def _call_pause(self):
        req = MovePause.Request()
        self.get_logger().warn(f"[MovePause] request send ({self.srv_pause})")
        fut = self.cli_pause.call_async(req)
        fut.add_done_callback(lambda f: self._log_future("MovePause", f))

    def _call_resume(self):
        req = MoveResume.Request()
        self.get_logger().warn(f"[MoveResume] request send ({self.srv_resume})")
        fut = self.cli_resume.call_async(req)
        fut.add_done_callback(lambda f: self._log_future("MoveResume", f))

    def _cb_cmd(self, msg: String):
        cmd = msg.data.strip().lower()
        self.get_logger().warn(f"[CMD] recv='{cmd}'")

        if cmd == "stop":
            self.get_logger().warn("[CMD] stop -> MoveStop(quick=1)")
            self._call_move_stop(1)

        elif cmd == "pause":
            self.get_logger().warn("[CMD] pause -> MovePause()")
            self._call_pause()

        elif cmd == "resume":
            self.get_logger().warn("[CMD] resume -> MoveResume()")
            self._call_resume()

        elif cmd == "reset":
            self.get_logger().warn("[CMD] reset -> MoveStop(quick=1)")
            self._call_move_stop(1)

        else:
            self.get_logger().info("[CMD] ignored (handled by task layer or unknown)")


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