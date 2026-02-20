#!/usr/bin/env python3
# tilemate_main/scraper_motion_node.py

import time
import traceback

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, Float64, String

import DR_init
from tilemate_main.robot_config import RobotConfig


class _GripperClient:
    def __init__(self, node: Node):
        self._node = node
        self._pub = node.create_publisher(Float64, "/gripper/width_m", 10)

    def set_width(self, width_m: float):
        msg = Float64()
        msg.data = float(width_m)
        self._pub.publish(msg)
        self._node.get_logger().info(f"[GRIPPER->CMD] width_m={msg.data:.4f}")


class ScraperMotionNode(Node):
    """
    Inputs:
      - /scraper/run_once (Int32 token)
      - /task/pause (Bool)
      - /task/stop_soft (Bool)

    Outputs:
      - /scraper/status (String): "done:<tok>" or "error:<tok>:<msg>"
      - /scraper/step (Int32), /scraper/state (String)
    """

    STEP_PREPARE  = 0
    STEP_GRIPPING = 1
    STEP_COATING  = 2
    STEP_FINISH   = 3

    def __init__(self, cfg: RobotConfig, boot_node: Node):
        super().__init__("scraper_motion_node", namespace=cfg.robot_id)
        self.cfg = cfg
        self._boot_node = boot_node

        self._pause = False
        self._stop_soft = False
        self._pending_token = None
        self._running = False

        # pubs
        self.pub_status = self.create_publisher(String, "/scraper/status", 10)
        self.pub_state  = self.create_publisher(String, "/robot/state", 10)
        self.pub_step   = self.create_publisher(Int32,  "/scraper/step", 10)

        # subs
        self.create_subscription(Int32, "/scraper/run_once", self._cb_run_once, 10)
        self.create_subscription(Bool,  "/task/pause", self._cb_pause, 10)
        self.create_subscription(Bool,  "/task/stop_soft", self._cb_stop_soft, 10)

        self.gripper = _GripperClient(self)

        self._initialize_robot()
        self._set_scraper_status(self.STEP_PREPARE, "대기(run_once 기다리는 중)")

        self.get_logger().info("ScraperMotionNode ready: sub /scraper/run_once")



    # -----------------
    # init / helpers
    # -----------------
    def _initialize_robot(self):
        from DSR_ROBOT2 import set_tool, set_tcp, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS, set_robot_mode
        self.get_logger().info("[SCRAPER] initialize_robot()")
        set_robot_mode(ROBOT_MODE_MANUAL)
        set_tool(self.cfg.tool)
        set_tcp(self.cfg.tcp)
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        time.sleep(1.0)

    def _set_scraper_status(self, step: int, state: str):
        m_step = Int32()
        m_step.data = int(step)
        m_state = String()
        m_state.data = str(state)
        self.pub_step.publish(m_step)
        self.pub_state.publish(m_state)
        self.get_logger().info(f"[SCRAPER] step={m_step.data} state='{m_state.data}'")

    def _publish_status(self, s: str):
        m = String()
        m.data = s
        self.pub_status.publish(m)
        self.get_logger().info(f"[SCRAPER->STATUS] {m.data}")

    def _wait_if_paused(self):
        if self._pause:
            self._set_scraper_status(self.STEP_PREPARE, "일시정지(pause)")
        while rclpy.ok() and self._pause and not self._stop_soft:
            time.sleep(0.05)

    def _checkpoint(self) -> bool:
        if self._stop_soft:
            return False
        self._wait_if_paused()
        if self._stop_soft:
            return False
        return True

    # -----------------
    # callbacks
    # -----------------
    def _cb_run_once(self, msg: Int32):
        if self._running:
            self.get_logger().warn("[SCRAPER] run_once ignored (already running)")
            return
        self._pending_token = int(msg.data)
        self.get_logger().info(f"[SCRAPER] received token={self._pending_token}")

    def _cb_pause(self, msg: Bool):
        self._pause = bool(msg.data)
        self.get_logger().warn(f"[SCRAPER] pause={self._pause}")

    def _cb_stop_soft(self, msg: Bool):
        self._stop_soft = bool(msg.data)
        self.get_logger().warn(f"[SCRAPER] stop_soft={self._stop_soft}")

    # -----------------
    # tick
    # -----------------
    def tick(self):
        if self._pending_token is None or self._running:
            return

        tok = self._pending_token
        self._pending_token = None

        if self._stop_soft:
            self.get_logger().warn("[SCRAPER] stop_soft=True -> skip token")
            return

        self._running = True
        try:
            self._wait_if_paused()
            if self._stop_soft:
                self._publish_status(f"error:{tok}:aborted(stop_soft)")
                return

            self.get_logger().info(f"[SCRAPER] run_once start token={tok}")
            ok = self._perform_cycle()

            if ok and not self._stop_soft:
                self._publish_status(f"done:{tok}")
            else:
                self._publish_status(f"error:{tok}:aborted/failed")

        except Exception as e:
            self.get_logger().error(f"[SCRAPER] exception: {e}")
            self.get_logger().error(traceback.format_exc())
            self._publish_status(f"error:{tok}:{e}")

        finally:
            self._running = False
            if not self._stop_soft:
                self._set_scraper_status(self.STEP_PREPARE, "대기(run_once 기다리는 중)")

    # -----------------
    # DSR motions
    # -----------------
    def _perform_cycle(self) -> bool:
        from DSR_ROBOT2 import posx, posj, movej, movel

        def set_gripper(w: float):
            self.gripper.set_width(w)
            time.sleep(0.05)

        JReady = [0, 0, 90, 0, 90, 0]

        pre = posx([634.6129, 70.1825, 216.4523, 52.0011, 179.0943, 52.3476])
        grasp = posx([635.6555, 69.9934, 156.9518, 122.9743, 179.7988, 123.1659])
        place = posx([635.6555, 69.9934, 186.9518, 122.9743, 179.7988, 123.1659])

        mid = posx([480.8698, 68.9976, 167.2608, 59.9196, 179.1564, 60.5511])
        p5 = posj([6.65, 16.37, 75.84, 0.56, 87.29, 95.63])

        tilt_right = posx([465.40, -34.13, 160.68, 92.35, -142.05, 179.91])
        pos7 = posx([465.40, 250.13, 160.68, 92.35, -142.05, 179.91])
        tilt_left = posx([462.86, 274.11, 160.68, 93.23, 148.62, -179.15])
        pos9 = posx([528.40, -34.13, 160.68, 93.23, 148.62, -179.15])

        # 준비
        self._set_scraper_status(self.STEP_PREPARE, "파지 준비전")
        movej(JReady, vel=self.cfg.vel, acc=self.cfg.acc)
        set_gripper(0.060)
        time.sleep(2.0)
        if not self._checkpoint(): return False

        # 파지
        self._set_scraper_status(self.STEP_GRIPPING, "파지중")
        movel(pre, vel=60, acc=60); time.sleep(1.0)
        if not self._checkpoint(): return False
        movel(grasp, vel=60, acc=60); time.sleep(1.0)
        if not self._checkpoint(): return False
        set_gripper(0.003); time.sleep(4.0)
        if not self._checkpoint(): return False
        movel(pre, vel=60, acc=60); time.sleep(1.0)
        if not self._checkpoint(): return False

        # 이동
        movel(mid, vel=60, acc=60); time.sleep(1.0)
        if not self._checkpoint(): return False
        movej(p5, vel=40, acc=40); time.sleep(1.0)
        if not self._checkpoint(): return False

        # 도포
        self._set_scraper_status(self.STEP_COATING, "도포중")
        movel(tilt_right, vel=40, acc=40); time.sleep(1.0)
        if not self._checkpoint(): return False
        movel(pos7, vel=40, acc=40); time.sleep(1.0)
        if not self._checkpoint(): return False
        movel(tilt_left, vel=20, acc=20); time.sleep(1.0)
        if not self._checkpoint(): return False
        movel(pos9, vel=40, acc=40); time.sleep(1.0)
        if not self._checkpoint(): return False

        # 반납
        self._set_scraper_status(self.STEP_FINISH, "끝")
        movel(pre, vel=60, acc=60); time.sleep(1.0)
        if not self._checkpoint(): return False
        movel(place, vel=60, acc=60); time.sleep(1.0)
        if not self._checkpoint(): return False
        set_gripper(0.040); time.sleep(4.0)
        if not self._checkpoint(): return False
        movel(pre, vel=60, acc=60); time.sleep(1.0)
        if not self._checkpoint(): return False

        return True


def main(args=None):
    rclpy.init(args=args)
    cfg = RobotConfig()

    # ✅ DSR_ROBOT2 import 전에 boot node 먼저 만들고 DR_init.__dsr__node 세팅
    boot = rclpy.create_node("dsr_boot_scraper", namespace=cfg.robot_id)
    DR_init.__dsr__id = cfg.robot_id
    DR_init.__dsr__model = cfg.robot_model
    DR_init.__dsr__node = boot

    import DSR_ROBOT2  # noqa: F401

    node = ScraperMotionNode(cfg, boot)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.tick()
    finally:
        try:
            node.destroy_node()
            boot.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()