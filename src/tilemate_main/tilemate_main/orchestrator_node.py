#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Bool

class OrchestratorNode(Node):
    """
    start 흐름:
      scraper/run_once -> scraper/status done -> tile/run_once -> tile/status done

    /robot/step Int32
      0: 초기상태(준비중)
      1: 접착제 파지
      2: 접착제 도포
      3: 타일 파지
      4: 타일 배치
    """
    STEP_READY        = 0
    STEP_GLUE_PICK    = 1
    STEP_GLUE_SPREAD  = 2
    STEP_TILE_PICK    = 3
    STEP_TILE_PLACE   = 4

    def __init__(self):
        super().__init__("orchestrator_node")

        self._busy = False
        self._token = 0
        self._phase = "IDLE"   # IDLE / SCRAPER / TILE

        self.pub_scraper = self.create_publisher(Int32, "/scraper/run_once", 10)
        self.pub_tile    = self.create_publisher(Int32, "/tile/run_once", 10)

        # 공통 제어 브로드캐스트(선택)
        self.pub_pause     = self.create_publisher(Bool, "/task/pause", 10)
        self.pub_stop_soft = self.create_publisher(Bool, "/task/stop_soft", 10)

        # step 토픽
        self.pub_step = self.create_publisher(Int32, "/robot/step", 10)
        self._set_step(self.STEP_READY)

        self.create_subscription(String, "/robot/command", self._cb_cmd, 10)
        self.create_subscription(String, "/scraper/status", self._cb_scraper_status, 10)
        self.create_subscription(String, "/tile/status", self._cb_tile_status, 10)

        self.get_logger().info("OrchestratorNode ready: sub /robot/command, /scraper/status, /tile/status")

    def _set_step(self, step: int):
        m = Int32()
        m.data = int(step)
        self.pub_step.publish(m)
        self.get_logger().info(f"[ORCH] /robot/step={m.data}")

    def _next_token(self):
        self._token += 1
        return self._token

    def _publish_bool(self, pub, v: bool):
        m = Bool()
        m.data = bool(v)
        pub.publish(m)

    def _start_scraper(self):
        # 1) 접착제 파지
        self._set_step(self.STEP_GLUE_PICK)

        tok = self._next_token()
        m = Int32()
        m.data = tok
        self.pub_scraper.publish(m)

        self._phase = "SCRAPER"
        self._busy = True
        self.get_logger().info(f"[ORCH] start scraper token={tok}")

        # 2) 접착제 도포 (정확한 분리는 scraper의 phase가 있어야 함)
        self._set_step(self.STEP_GLUE_SPREAD)

    def _start_tile(self, tok: int):
        # 3) 타일 파지 (tile 동작 직전에)
        self._set_step(self.STEP_TILE_PICK)

        m = Int32()
        m.data = tok
        self.pub_tile.publish(m)
        self._phase = "TILE"
        self.get_logger().info(f"[ORCH] start tile token={tok}")

        # 4) 타일 배치 (tile 동작 중)
        self._set_step(self.STEP_TILE_PLACE)

    def _finish_all(self):
        self._busy = False
        self._phase = "IDLE"
        self._set_step(self.STEP_READY)

    def _cb_cmd(self, msg: String):
        cmd = msg.data.strip().lower()

        if cmd == "start":
            if self._busy:
                self.get_logger().warn("[ORCH] start ignored (busy)")
                return
            self._publish_bool(self.pub_stop_soft, False)
            self._publish_bool(self.pub_pause, False)
            self._start_scraper()

        elif cmd == "stop":
            self.get_logger().warn("[ORCH] stop_soft True")
            self._publish_bool(self.pub_stop_soft, True)
            self._finish_all()

        elif cmd == "pause":
            self.get_logger().warn("[ORCH] pause True")
            self._publish_bool(self.pub_pause, True)

        elif cmd == "resume":
            self.get_logger().info("[ORCH] pause False")
            self._publish_bool(self.pub_pause, False)

        elif cmd == "reset":
            self.get_logger().warn("[ORCH] reset -> stop_soft True + IDLE")
            self._publish_bool(self.pub_stop_soft, True)
            self._finish_all()

        else:
            self.get_logger().warn(f"[ORCH] unknown command: {cmd}")

    def _parse_status(self, s: str):
        # "done:3" / "error:3:msg"
        parts = s.strip().split(":", 2)
        if len(parts) < 2:
            return None, None, None
        state = parts[0]
        try:
            tok = int(parts[1])
        except ValueError:
            return None, None, None
        msg = parts[2] if len(parts) == 3 else ""
        return state, tok, msg

    def _cb_scraper_status(self, msg: String):
        state, tok, emsg = self._parse_status(msg.data)
        if state is None:
            return
        if self._phase != "SCRAPER":
            return

        if state == "done":
            self.get_logger().info(f"[ORCH] scraper done token={tok} -> start tile")
            self._start_tile(tok)

        elif state == "error":
            self.get_logger().error(f"[ORCH] scraper error token={tok}: {emsg}")
            self._finish_all()

    def _cb_tile_status(self, msg: String):
        state, tok, emsg = self._parse_status(msg.data)
        if state is None:
            return
        if self._phase != "TILE":
            return

        if state == "done":
            self.get_logger().info(f"[ORCH] tile done token={tok} -> ALL DONE")
            self._finish_all()

        elif state == "error":
            self.get_logger().error(f"[ORCH] tile error token={tok}: {emsg}")
            self._finish_all()


def main(args=None):
    rclpy.init(args=args)
    node = OrchestratorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == "__main__":
    main()
