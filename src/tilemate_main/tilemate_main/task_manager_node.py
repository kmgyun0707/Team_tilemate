#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Bool


class TaskManagerNode(Node):
    """
    단일 최상단 상태관리/오케스트레이션 노드.

    start 흐름 (분리 반영):
      scraper/run_once(token)
        -> scraper/status done:token
        -> tile/run_once(token)               # (타일 배치만: pick/place + suction tool return까지)
        -> tile/status done:token
        -> tile/inspect/run_once(token)       # 단차 측정
        -> tile/inspect/status done:token
        -> tile/compact/run_once(token)       # 압착
        -> tile/compact/status done:token

    /robot/step (UI/모니터링용 단일 정의)
      0: READY
      1: GLUE_PICK
      2: GLUE_SPREAD
      3: TILE_PICK
      4: TILE_PLACE
      5: TILE_INSPECT
      6: TILE_COMPACT
      7: FINISHED
    """

    STEP_READY = 0
    STEP_GLUE_PICK = 1
    STEP_GLUE_SPREAD = 2
    STEP_TILE_PICK = 3
    STEP_TILE_PLACE = 4
    STEP_TILE_INSPECT = 5
    STEP_TILE_COMPACT = 6
    STEP_FINISHED = 7

    def __init__(self):
        super().__init__("task_manager_node")

        # State Machine
        self._busy = False
        self._phase = "IDLE"  # IDLE / SCRAPER / TILE / INSPECT / COMPACT
        self._active_token = 0
        self._robot_step_last = None

        # 중복 start 방지(각 단계별)
        self._tile_started_token = None
        self._inspect_started_token = None
        self._compact_started_token = None

        # publishers ------------------------------------------------------------
        self.pub_scraper = self.create_publisher(Int32, "/scraper/run_once", 10)
        self.pub_tile = self.create_publisher(Int32, "/tile/run_once", 10)

        # ✅ 분리 노드 트리거
        self.pub_tile_inspect = self.create_publisher(Int32, "/tile/inspect/run_once", 10)
        self.pub_tile_compact = self.create_publisher(Int32, "/tile/compact/run_once", 10)

        # 내부 제어용 토픽
        self.pub_reset = self.create_publisher(Bool, "/task/reset", 10)
        self.pub_pause = self.create_publisher(Bool, "/task/pause", 10)
        self.pub_stop_soft = self.create_publisher(Bool, "/task/stop_soft", 10)

        # UI step
        self.pub_step = self.create_publisher(Int32, "/robot/step", 10)
        self._set_step(self.STEP_READY)

        # subscribers -----------------------------------------------------------
        self.create_subscription(String, "/robot/command", self._cb_cmd, 10)

        self.create_subscription(String, "/scraper/status", self._cb_scraper_status, 10)
        self.create_subscription(String, "/tile/status", self._cb_tile_status, 10)

        # ✅ 분리 노드 status 구독
        self.create_subscription(String, "/tile/inspect/status", self._cb_inspect_status, 10)
        self.create_subscription(String, "/tile/compact/status", self._cb_compact_status, 10)

        # 각 노드 step 구독
        self.create_subscription(Int32, "/scraper/step", self._cb_scraper_step, 10)

        # ✅ 배치 노드(기존 /tile/step)는 3,4까지만 쓰는 걸 권장 (pick/place)
        self.create_subscription(Int32, "/tile/step", self._cb_tile_step, 10)

        # (선택) inspect/compact도 /tile/step를 같이 찍는다면 그대로 /tile/step를 통해 매핑 가능.
        # 지금 boilerplate는 inspect가 /tile/step=5, compact가 /tile/step=6/7를 publish하는 구조라
        # 별도 step 구독이 필요 없다. (이미 /tile/step 하나로 처리됨)

        self.get_logger().info("TaskManagerNode ready")

    # -----------------
    # publish helpers
    # -----------------
    def _set_step(self, step: int):
        step = int(step)
        if self._robot_step_last == step:
            return
        self._robot_step_last = step

        m = Int32()
        m.data = step
        self.pub_step.publish(m)
        self.get_logger().info(f"[TASK] /robot/step={m.data}")

    def _pub_bool(self, pub, v: bool):
        m = Bool()
        m.data = bool(v)
        pub.publish(m)

    def _pulse_bool(self, pub):
        """latch 방지: True 후 바로 False로 펄스"""
        self._pub_bool(pub, True)
        self._pub_bool(pub, False)

    def _next_token(self) -> int:
        self._active_token += 1
        return self._active_token

    # -----------------
    # orchestrate actions
    # -----------------
    def _start_cycle(self):
        """새 사이클 시작: scraper -> tile -> inspect -> compact"""
        self._busy = True
        self._phase = "SCRAPER"

        # 새 사이클이면 단계별 중복방지 토큰 리셋
        self._tile_started_token = None
        self._inspect_started_token = None
        self._compact_started_token = None

        tok = self._next_token()

        # 1) glue pick
        self._set_step(self.STEP_GLUE_PICK)

        m = Int32()
        m.data = tok
        self.pub_scraper.publish(m)
        self.get_logger().info(f"[TASK] start SCRAPER token={tok}")

    def _start_tile(self, tok: int):
        """scraper 완료 후 tile(배치) 시작"""
        if self._tile_started_token == tok:
            self.get_logger().warn(f"[TASK] tile already started for token={tok} -> ignore")
            return
        self._tile_started_token = tok

        self._phase = "TILE"
        self._set_step(self.STEP_TILE_PICK)

        m = Int32()
        m.data = tok
        self.pub_tile.publish(m)
        self.get_logger().info(f"[TASK] start TILE(PLACE) token={tok}")

    def _start_inspect(self, tok: int):
        """tile(배치) 완료 후 inspect 시작"""
        if self._inspect_started_token == tok:
            self.get_logger().warn(f"[TASK] inspect already started for token={tok} -> ignore")
            return
        self._inspect_started_token = tok

        self._phase = "INSPECT"
        self._set_step(self.STEP_TILE_INSPECT)

        m = Int32()
        m.data = tok
        self.pub_tile_inspect.publish(m)
        self.get_logger().info(f"[TASK] start TILE_INSPECT token={tok}")

    def _start_compact(self, tok: int):
        """inspect 완료 후 compact 시작"""
        if self._compact_started_token == tok:
            self.get_logger().warn(f"[TASK] compact already started for token={tok} -> ignore")
            return
        self._compact_started_token = tok

        self._phase = "COMPACT"
        self._set_step(self.STEP_TILE_COMPACT)

        m = Int32()
        m.data = tok
        self.pub_tile_compact.publish(m)
        self.get_logger().info(f"[TASK] start TILE_COMPACT token={tok}")

    def _finish_ok(self):
        self._busy = False
        self._phase = "IDLE"
        self._set_step(self.STEP_FINISHED)

    def _finish_abort(self, reset_step: bool = True):
        self._busy = False
        self._phase = "IDLE"
        if reset_step:
            self._set_step(self.STEP_READY)

    # -----------------
    # status parsing
    # -----------------
    def _parse_status(self, s: str):
        # "done:3" / "error:3:msg" / "stopped:3:ckpt"
        parts = s.strip().split(":", 2)
        if len(parts) < 2:
            return None, None, None

        state = parts[0].strip().lower()
        try:
            tok = int(parts[1])
        except ValueError:
            return None, None, None

        msg = parts[2] if len(parts) == 3 else ""
        return state, tok, msg

    def _is_active_token(self, tok: int) -> bool:
        return tok == self._active_token

    # -----------------
    # callbacks
    # -----------------
    def _cb_cmd(self, msg: String):
        cmd = msg.data.strip().lower()

        if cmd in ("start", "once"):
            if self._busy:
                self.get_logger().warn("[TASK] start ignored (busy)")
                return

            # pause/stop latch 해제
            self._pub_bool(self.pub_pause, False)
            self._pub_bool(self.pub_stop_soft, False)

            self._start_cycle()
            return

        if cmd == "pause":
            self.get_logger().warn("[TASK] pause True")
            self._pub_bool(self.pub_pause, True)
            return

        if cmd == "resume":
            self.get_logger().info("[TASK] pause False")
            self._pub_bool(self.pub_pause, False)
            return

        if cmd == "stop":
            self.get_logger().warn("[TASK] stop_soft PULSE + abort")
            self._finish_abort(reset_step=False)
            self._pub_bool(self.pub_pause, False)
            self._pulse_bool(self.pub_stop_soft)
            return

        if cmd == "reset":
            self.get_logger().warn("[TASK] reset requested")

            # 1) task 상태 IDLE + token invalidate
            self._finish_abort(reset_step=True)
            self._active_token += 1

            # 2) pause 해제
            self._pub_bool(self.pub_pause, False)

            # 3) 로봇 정지(펄스)
            self._pulse_bool(self.pub_stop_soft)

            # 4) reset 펄스
            self._pulse_bool(self.pub_reset)

            # 5) stop_soft latch 방지(명시적으로 False)
            self._pub_bool(self.pub_stop_soft, False)

            self.get_logger().warn("[TASK] reset -> READY")
            return

        self.get_logger().warn(f"[TASK] unknown command: {cmd}")

    def _cb_scraper_status(self, msg: String):
        state, tok, emsg = self._parse_status(msg.data)
        if state is None:
            return
        if not self._is_active_token(tok):
            return
        if not self._busy:
            return

        if state == "done":
            if self._phase == "SCRAPER":
                self.get_logger().info(f"[TASK] scraper done token={tok} -> start tile(place)")
                self._start_tile(tok)
            return

        if state == "error":
            self.get_logger().error(f"[TASK] scraper error token={tok}: {emsg}")
            self._finish_abort()
            return

        if state == "stopped":
            self.get_logger().warn(f"[TASK] scraper stopped token={tok}: {emsg}")
            self._finish_abort(reset_step=False)
            return

    def _cb_tile_status(self, msg: String):
        """tile = '타일 배치 노드' (inspect/compact 분리 후에는 여기서 FINISHED 하면 안됨)"""
        state, tok, emsg = self._parse_status(msg.data)
        if state is None:
            return
        if not self._is_active_token(tok):
            return
        if not self._busy or self._phase != "TILE":
            return

        if state == "done":
            self.get_logger().info(f"[TASK] tile(place) done token={tok} -> start inspect")
            self._start_inspect(tok)
            return

        if state == "error":
            self.get_logger().error(f"[TASK] tile(place) error token={tok}: {emsg}")
            self._finish_abort()
            return

        if state == "stopped":
            self.get_logger().warn(f"[TASK] tile(place) stopped token={tok}: {emsg}")
            self._finish_abort(reset_step=False)
            return

    def _cb_inspect_status(self, msg: String):
        state, tok, emsg = self._parse_status(msg.data)
        if state is None:
            return
        if not self._is_active_token(tok):
            return
        if not self._busy or self._phase != "INSPECT":
            return

        if state == "done":
            self.get_logger().info(f"[TASK] inspect done token={tok} -> start compact")
            self._start_compact(tok)
            return

        if state == "error":
            self.get_logger().error(f"[TASK] inspect error token={tok}: {emsg}")
            self._finish_abort()
            return

        if state == "stopped":
            self.get_logger().warn(f"[TASK] inspect stopped token={tok}: {emsg}")
            self._finish_abort(reset_step=False)
            return

    def _cb_compact_status(self, msg: String):
        state, tok, emsg = self._parse_status(msg.data)
        if state is None:
            return
        if not self._is_active_token(tok):
            return
        if not self._busy or self._phase != "COMPACT":
            return

        if state == "done":
            self.get_logger().info(f"[TASK] compact done token={tok} -> FINISHED")
            self._finish_ok()
            return

        if state == "error":
            self.get_logger().error(f"[TASK] compact error token={tok}: {emsg}")
            self._finish_abort()
            return

        if state == "stopped":
            self.get_logger().warn(f"[TASK] compact stopped token={tok}: {emsg}")
            self._finish_abort(reset_step=False)
            return

    # -----------------
    # step callbacks
    # -----------------
    def _cb_scraper_step(self, msg: Int32):
        if not self._busy:
            return
        if self._phase != "SCRAPER":
            return

        s = int(msg.data)
        if s == 1:
            self._set_step(self.STEP_GLUE_PICK)
        elif s == 2:
            self._set_step(self.STEP_GLUE_SPREAD)

    def _cb_tile_step(self, msg: Int32):
        """
        /tile/step 통합 매핑 (분리 후에도 유지)
          3 TILE_PICK   -> /robot/step 3
          4 TILE_PLACE  -> /robot/step 4
          5 INSPECT     -> /robot/step 5
          6 COMPACT     -> /robot/step 6
          7 DONE        -> /robot/step 7
        """
        if not self._busy:
            return

        s = int(msg.data)

        # phase gate: 각 단계에서만 반영 (UI step 꼬임 방지)
        if self._phase == "TILE":
            if s == 3:
                self._set_step(self.STEP_TILE_PICK)
            elif s == 4:
                self._set_step(self.STEP_TILE_PLACE)

        elif self._phase == "INSPECT":
            if s == 5:
                self._set_step(self.STEP_TILE_INSPECT)

        elif self._phase == "COMPACT":
            if s == 6:
                self._set_step(self.STEP_TILE_COMPACT)
            elif s == 7:
                self._set_step(self.STEP_FINISHED)


def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()
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