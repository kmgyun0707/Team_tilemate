#!/usr/bin/env python3
# tilemate_main/interrupt_node.py

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

from dsr_msgs2.srv import MoveJoint, GetRobotState, SetRobotControl, SetRobotMode, MoveStop


class InterruptNode(Node):
    # --- state enums (manual 기준) ---
    STATE_STANDBY   = 1
    STATE_SAFE_OFF  = 3
    STATE_TEACHING  = 4
    STATE_SAFE_STOP = 5
    STATE_EMG_STOP  = 6

    # --- control enums ---
    CONTROL_RESET_SAFE_STOP = 2
    CONTROL_RESET_SAFE_OFF  = 3

    # --- mode enums ---
    ROBOT_MODE_AUTONOMOUS = 1

    def __init__(self):
        super().__init__("interrupt_node")

        # pubs
        self.pub_pause = self.create_publisher(Bool, "/task/pause", 10)
        self.pub_stop_soft = self.create_publisher(Bool, "/task/stop_soft", 10)

        # resume triggers (Option B)
        self.pub_scraper_resume = self.create_publisher(Bool, "/scraper/resume", 10)
        self.pub_tile_resume = self.create_publisher(Bool, "/tile/resume", 10)

        # subs
        self.create_subscription(String, "/robot/command", self._cb_cmd, 10)
        self.create_subscription(Bool, "/task/reset", self._cb_reset, 10)

        # service clients
        self.cli_move_joint = self.create_client(MoveJoint, "/dsr01/motion/move_joint")
        self.cli_move_stop  = self.create_client(MoveStop,  "/dsr01/motion/move_stop")
        self.cli_get_state  = self.create_client(GetRobotState, "/dsr01/system/get_robot_state")
        self.cli_set_ctrl   = self.create_client(SetRobotControl, "/dsr01/system/set_robot_control")
        self.cli_set_mode   = self.create_client(SetRobotMode,  "/dsr01/system/set_robot_mode")

        # JReady (deg)
        self.JREADY = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

        # reset state machine
        self._reset_state = "IDLE"      # IDLE/SET_AUTO/GET_STATE/SET_CTRL/MOVEJ
        self._reset_deadline = 0.0
        self._pending_future = None
        self._pending_tag = ""
        self._reset_next_control = 0

        # --- STOP state machine (MoveStop FIRST) ---
        self._stop_state = "IDLE"       # IDLE/CALL_WAIT/RETRY_WAIT
        self._stop_deadline = 0.0
        self._stop_future = None
        self._stop_retry_left = 0
        self._stop_retry_next_t = 0.0
        self._stop_stop_mode = 1  # 1=일반 stop (프로젝트에서 쓰던 값)

        # timer: 50ms tick
        self._timer = self.create_timer(0.05, self._tick)

        self.get_logger().warn("### INTERRUPT_NODE: STOP=MoveStop FIRST -> then stop_soft latch ###")
        self.get_logger().info("InterruptNode ready: sub /robot/command, /task/reset")

    # -----------------------------
    # small helpers
    # -----------------------------
    def _pub_bool(self, pub, v: bool, tag: str):
        m = Bool()
        m.data = bool(v)
        pub.publish(m)
        self.get_logger().warn(f"[PUB] {tag}={m.data}")

    # -----------------------------
    # unified tick
    # -----------------------------
    def _tick(self):
        self._tick_stop()
        self._tick_reset()

    # -----------------------------
    # STOP state machine tick (MoveStop FIRST)
    # -----------------------------
    def _arm_stop_sm(self, stop_mode: int = 1, retries: int = 2, timeout_s: float = 1.5):
        """
        stop 명령: MoveStop을 먼저 호출해 컨트롤러 모션을 선점(preempt)하고,
        그 다음 stop_soft=True로 앱 레벨 재발진을 막는다.
        """
        # 이미 stop 진행 중이면 중복 arm은 무시(로그만)
        if self._stop_state != "IDLE":
            self.get_logger().warn(f"[INT] stop ignored (stop_sm busy state={self._stop_state})")
            return

        self._stop_stop_mode = int(stop_mode)
        self._stop_retry_left = int(retries)
        self._stop_deadline = time.time() + float(timeout_s)
        self._stop_future = None
        self._stop_state = "CALL_WAIT"
        self.get_logger().warn(f"[INT][STOP] armed stop_mode={self._stop_stop_mode} retries={self._stop_retry_left}")

        # (중요) pause는 풀어주고(stop이 pause gate 때문에 늦어지는 것 방지)
        self._pub_bool(self.pub_pause, False, "/task/pause")

        # 첫 호출 즉시 발행
        self._fire_move_stop()

    def _fire_move_stop(self):
        # service ready 체크 (non-blocking)
        if not self.cli_move_stop.service_is_ready():
            self.cli_move_stop.wait_for_service(timeout_sec=0.0)
            self.get_logger().warn("[INT][STOP] MoveStop service not ready yet")
            return

        req = MoveStop.Request()
        req.stop_mode = int(self._stop_stop_mode)
        self._stop_future = self.cli_move_stop.call_async(req)
        self.get_logger().warn(f"[INT][STOP] MoveStop request send stop_mode={req.stop_mode}")

    def _tick_stop(self):
        if self._stop_state == "IDLE":
            return

        # timeout
        if time.time() > self._stop_deadline:
            self.get_logger().error("[INT][STOP] timeout -> still latch stop_soft True")
            # 서비스콜 실패해도 앱 레벨 stop_soft는 걸어야 함
            self._pub_bool(self.pub_stop_soft, True, "/task/stop_soft")
            self._stop_state = "IDLE"
            self._stop_future = None
            return

        # retry wait
        if self._stop_state == "RETRY_WAIT":
            if time.time() >= self._stop_retry_next_t:
                self._stop_state = "CALL_WAIT"
                self._fire_move_stop()
            return

        # CALL_WAIT: future 결과 확인
        if self._stop_state == "CALL_WAIT":
            if self._stop_future is None:
                # 서비스 준비 안 돼서 못 쐈으면 계속 시도
                self._fire_move_stop()
                return

            if not self._stop_future.done():
                return

            try:
                res = self._stop_future.result()
            except Exception as e:
                self.get_logger().error(f"[INT][STOP] MoveStop future exception: {e}")
                res = None

            ok = bool(getattr(res, "success", False)) if res is not None else False
            self.get_logger().warn(f"[INT][STOP] MoveStop success={ok}")

            if ok:
                # ✅ 컨트롤러 모션 선점 성공 -> stop_soft latch
                self._pub_bool(self.pub_stop_soft, True, "/task/stop_soft")
                self._stop_state = "IDLE"
                self._stop_future = None
                return

            # 실패면 재시도
            if self._stop_retry_left > 0:
                self._stop_retry_left -= 1
                self._stop_future = None
                self._stop_state = "RETRY_WAIT"
                self._stop_retry_next_t = time.time() + 0.08  # 80ms 후 재시도
                self.get_logger().warn(f"[INT][STOP] retry left={self._stop_retry_left}")
                return

            # 재시도 끝 -> 그래도 stop_soft는 걸어야 함
            self.get_logger().error("[INT][STOP] MoveStop failed -> latch stop_soft True anyway")
            self._pub_bool(self.pub_stop_soft, True, "/task/stop_soft")
            self._stop_state = "IDLE"
            self._stop_future = None
            return

    # -----------------------------
    # reset state machine tick (기존 그대로)
    # -----------------------------
    def _arm_reset_sm(self):
        """콜백에서는 '요청'만 세팅. 실제 서비스콜/대기는 타이머에서 처리."""
        if self._reset_state != "IDLE":
            self.get_logger().warn(f"[INT] reset ignored (state={self._reset_state})")
            return

        # pause 해제 + stop_soft True
        self._pub_bool(self.pub_pause, False, "/task/pause")
        self._pub_bool(self.pub_stop_soft, True, "/task/stop_soft")

        # reset 시퀀스 시작
        self._reset_deadline = time.time() + 8.0   # 전체 타임아웃
        self._reset_state = "SET_AUTO"
        self._pending_future = None
        self._pending_tag = ""
        self._reset_next_control = 0
        self.get_logger().warn("[INT] reset SM armed")

    def _reset_fail(self, reason: str):
        self.get_logger().error(f"[INT] reset FAIL: {reason}")
        self._pub_bool(self.pub_stop_soft, False, "/task/stop_soft")
        self._reset_state = "IDLE"
        self._pending_future = None
        self._pending_tag = ""

    def _reset_done(self):
        self.get_logger().warn("[INT] reset DONE -> standby")
        self._pub_bool(self.pub_stop_soft, False, "/task/stop_soft")
        self._reset_state = "IDLE"
        self._pending_future = None
        self._pending_tag = ""

    def _tick_reset(self):
        if self._reset_state == "IDLE":
            return

        if time.time() > self._reset_deadline:
            self._reset_fail("timeout")
            return

        # 1) pending future 결과 처리
        if self._pending_future is not None:
            if not self._pending_future.done():
                return

            fut = self._pending_future
            tag = self._pending_tag
            self._pending_future = None
            self._pending_tag = ""

            try:
                res = fut.result()
            except Exception as e:
                self._reset_fail(f"future '{tag}' exception: {e}")
                return

            if tag == "set_mode":
                self.get_logger().warn(f"[SetRobotMode] success={getattr(res, 'success', None)}")
                self._reset_state = "GET_STATE"
                return

            if tag == "get_state":
                ok = bool(getattr(res, "success", False))
                st = int(getattr(res, "robot_state", -1))

                if not ok:
                    self.get_logger().warn("[GetRobotState] failed")
                    self._reset_state = "GET_STATE"
                    return

                self.get_logger().warn(f"[INT] robot_state={st}")

                if st == self.STATE_STANDBY:
                    self._reset_state = "MOVEJ"
                    return

                if st == self.STATE_TEACHING:
                    self._reset_fail("TEACHING: switch to AUTO/REMOTE")
                    return

                if st == self.STATE_EMG_STOP:
                    self._reset_fail("EMERGENCY_STOP: release E-stop")
                    return

                if st == self.STATE_SAFE_STOP:
                    self._reset_next_control = self.CONTROL_RESET_SAFE_STOP
                    self._reset_state = "SET_CTRL"
                    return

                if st == self.STATE_SAFE_OFF:
                    self._reset_next_control = self.CONTROL_RESET_SAFE_OFF
                    self._reset_state = "SET_CTRL"
                    return

                self._reset_state = "GET_STATE"
                return

            if tag == "set_ctrl":
                self.get_logger().warn(f"[SetRobotControl] success={getattr(res, 'success', None)}")
                time.sleep(0.1)
                self._reset_state = "GET_STATE"
                return

            if tag == "movej":
                ok = bool(getattr(res, "success", False))
                self.get_logger().warn(f"[MoveJoint] success={ok}")
                if not ok:
                    self._reset_fail("MoveJoint failed")
                    return
                self._reset_done()
                return

            self._reset_fail(f"unknown future tag: {tag}")
            return

        # 2) pending future가 없으면, state에 따라 새로운 request를 발행
        if self._reset_state == "SET_AUTO":
            if not self.cli_set_mode.service_is_ready():
                self.cli_set_mode.wait_for_service(timeout_sec=0.0)
                return
            req = SetRobotMode.Request()
            req.robot_mode = self.ROBOT_MODE_AUTONOMOUS
            self._pending_future = self.cli_set_mode.call_async(req)
            self._pending_tag = "set_mode"
            return

        if self._reset_state == "GET_STATE":
            if not self.cli_get_state.service_is_ready():
                self.cli_get_state.wait_for_service(timeout_sec=0.0)
                return
            self._pending_future = self.cli_get_state.call_async(GetRobotState.Request())
            self._pending_tag = "get_state"
            return

        if self._reset_state == "SET_CTRL":
            if not self.cli_set_ctrl.service_is_ready():
                self.cli_set_ctrl.wait_for_service(timeout_sec=0.0)
                return
            req = SetRobotControl.Request()
            req.robot_control = int(self._reset_next_control)
            self._pending_future = self.cli_set_ctrl.call_async(req)
            self._pending_tag = "set_ctrl"
            return

        if self._reset_state == "MOVEJ":
            if not self.cli_move_joint.service_is_ready():
                self.cli_move_joint.wait_for_service(timeout_sec=0.0)
                return
            req = MoveJoint.Request()
            req.pos = [float(v) for v in self.JREADY]
            req.vel = 30.0
            req.acc = 30.0
            req.time = 0.0
            req.sync_type = 0
            self._pending_future = self.cli_move_joint.call_async(req)
            self._pending_tag = "movej"
            return

        self._reset_fail(f"unknown state: {self._reset_state}")

    # -----------------------------
    # actions (non-reset)
    # -----------------------------
    def _do_stop(self):
        # ✅ MoveStop FIRST, then stop_soft latch
        self._arm_stop_sm(stop_mode=1, retries=2, timeout_s=1.5)

    def _do_resume_option_b(self):
        self._pub_bool(self.pub_stop_soft, False, "/task/stop_soft (first)")
        self._pub_bool(self.pub_pause, False, "/task/pause")
        self._pub_bool(self.pub_scraper_resume, True, "/scraper/resume")
        self._pub_bool(self.pub_tile_resume, True, "/tile/resume")

    # -----------------------------
    # callbacks
    # -----------------------------
    def _cb_cmd(self, msg: String):
        cmd = (msg.data or "").strip().lower()
        self.get_logger().warn(f"[CMD] recv='{cmd}'")

        if cmd in ("stop", "stop_soft", "halt"):
            self._do_stop()
        elif cmd == "reset":
            self._arm_reset_sm()
        elif cmd in ("resume", "continue"):
            self._do_resume_option_b()
        elif cmd == "pause":
            self._pub_bool(self.pub_pause, True, "/task/pause")
        elif cmd in ("unpause", "play"):
            self._pub_bool(self.pub_pause, False, "/task/pause")
        else:
            self.get_logger().warn(f"[CMD] unknown: {cmd}")

    def _cb_reset(self, msg: Bool):
        if not msg.data:
            return
        self.get_logger().warn("[INT] /task/reset received")
        self._arm_reset_sm()


def main(args=None):
    rclpy.init(args=args)
    node = InterruptNode()
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