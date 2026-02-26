#!/usr/bin/env python3
# tilemate_main/generic_job_node.py
#
# ✅ Generic Job Node Boilerplate (Option B: Stop -> Resume "앱 레벨 재시작")
# - run_once(token) → worker thread에서 _perform_cycle() 1회 실행
# - stop_soft → "stopped:<tok>:<ckpt>" 로 종료 + checkpoint 유지
# - resume(True) → 마지막 token + checkpoint로 재시작(restart from checkpoint)
# - pause → 앱 레벨 sleep gate
#
# ✅ 이 파일을 복사해서:
#   - 노드명/토픽명만 바꾸고
#   - _perform_cycle() 안에 DSR 시퀀스(또는 일반 로직)만 채우면 됨
#
# 예) check_level_motion_node / pypress_motion_node 등으로 그대로 확장 가능

import time
import traceback
import threading
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String

import DR_init
from tilemate_main.robot_config import RobotConfig


class GenericJobNode(Node):
    # -----------------
    # 상태(필요하면 확장)
    # -----------------
    STEP_IDLE    = 0
    STEP_RUNNING = 1
    STEP_DONE    = 2
    STEP_ERROR   = 9

    def __init__(
        self,
        cfg: RobotConfig,
        boot_node: Node,
        *,
        node_name: str = "generic_job_node",
        # I/O topics
        topic_run_once: str = "/job/run_once",
        topic_resume: str = "/job/resume",
        topic_pause: str = "/task/pause",
        topic_stop_soft: str = "/task/stop_soft",
        topic_status: str = "/job/status",
        topic_step: str = "/job/step",
        topic_state: str = "/robot/state",
    ):
        super().__init__(node_name, namespace=cfg.robot_id)

        self.cfg = cfg
        self._boot_node = boot_node

        # ---- flags ----
        self._pause = False
        self._stop_soft = False

        # ---- token/job ----
        self._pending_token: Optional[int] = None
        self._last_token: Optional[int] = None

        # ---- resume (Option B) ----
        self._resume_requested = False

        # ---- checkpoint ----
        # 형태는 자유: {"phase":"XXX", "i":N, ...}
        self._checkpoint: Optional[Dict[str, Any]] = None
        self._stopped = False

        # ---- worker state ----
        self._running = False
        self._worker = None
        self._worker_done = True
        self._worker_ok = False
        self._worker_tok: Optional[int] = None
        self._worker_err = ""  # "stopped" | error string

        # ---- pubs ----
        self.pub_status = self.create_publisher(String, topic_status, 10)
        self.pub_step   = self.create_publisher(Int32,  topic_step, 10)
        self.pub_state  = self.create_publisher(String, topic_state, 10)

        # ---- subs ----
        self.create_subscription(Int32, topic_run_once, self._cb_run_once, 10)
        self.create_subscription(Bool,  topic_resume,   self._cb_resume, 10)
        self.create_subscription(Bool,  topic_pause,    self._cb_pause, 10)
        self.create_subscription(Bool,  topic_stop_soft, self._cb_stop_soft, 10)

        # ---- robot init (필요 없으면 주석/삭제 가능) ----
        self._initialize_robot()

        self._set_status_step(self.STEP_IDLE, "작업명령 대기중")
        self.get_logger().info(f"{node_name} ready!!!")

    # -----------------
    # init / helpers
    # -----------------
    def _initialize_robot(self):
        """
        DSR 사용 노드면 보통 필요.
        DSR 미사용 노드면 이 함수 통째로 지워도 됨.
        """
        from DSR_ROBOT2 import set_tool, set_tcp, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS, set_robot_mode
        self.get_logger().info("[JOB] initialize_robot()")
        set_robot_mode(ROBOT_MODE_MANUAL)
        set_tool(self.cfg.tool)
        set_tcp(self.cfg.tcp)
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        time.sleep(0.2)

    def _publish_status(self, s: str):
        m = String()
        m.data = str(s)
        self.pub_status.publish(m)
        self.get_logger().info(f"[JOB->STATUS] {m.data}")

    def _set_status_step(self, step: int, state: str):
        m_step = Int32()
        m_step.data = int(step)
        m_state = String()
        m_state.data = str(state)
        self.pub_step.publish(m_step)
        self.pub_state.publish(m_state)
        self.get_logger().info(f"[JOB] step={m_step.data} state='{m_state.data}'")

    def _wait_if_paused(self):
        # Pause(일시정지, pause) = 앱 레벨 sleep gate
        if self._pause:
            self._set_status_step(self.STEP_IDLE, "일시정지(pause)")
        while rclpy.ok() and self._pause and not self._stop_soft:
            time.sleep(0.05)

    def _sleep_interruptible(self, sec: float, dt: float = 0.05) -> bool:
        t0 = time.time()
        while (time.time() - t0) < float(sec):
            if self._stop_soft:
                return False
            self._wait_if_paused()
            time.sleep(float(dt))
        return True

    def _check_abort(self) -> bool:
        # stop_soft(취소/정지) 체크
        if self._stop_soft:
            self.get_logger().warn("[JOB] stop requested (stop_soft=True)")
            return True
        self._wait_if_paused()
        return bool(self._stop_soft)

    # ---- checkpoint helpers ----
    def _set_ckpt(self, ckpt: Dict[str, Any]):
        """
        ckpt 예:
          {"phase": "PREPARE"}
          {"phase": "LOOP", "i": 3}
          {"phase": "FINISH"}
        """
        if self._checkpoint == ckpt:
            return
        self._checkpoint = dict(ckpt)
        self.get_logger().info(f"[JOB][CKPT] set {self._checkpoint_to_string()}")

    def _checkpoint_to_string(self) -> str:
        if not self._checkpoint:
            return "none"
        # 사람이 보기 좋게 phase 우선
        phase = self._checkpoint.get("phase", "none")
        rest = {k: v for k, v in self._checkpoint.items() if k != "phase"}
        if not rest:
            return f"{phase}"
        return f"{phase}:{rest}"

    # -----------------
    # callbacks
    # -----------------
    def _cb_run_once(self, msg: Int32):
        if self._running:
            self.get_logger().warn("[JOB] run_once ignored (already running)")
            return
        self._pending_token = int(msg.data)
        self.get_logger().info(f"[JOB] received token={self._pending_token}")

    def _cb_pause(self, msg: Bool):
        self._pause = bool(msg.data)
        self.get_logger().warn(f"[JOB] pause={self._pause}")

    def _cb_stop_soft(self, msg: Bool):
        self._stop_soft = bool(msg.data)
        self.get_logger().warn(f"[JOB] stop_soft={self._stop_soft}")

    def _cb_resume(self, msg: Bool):
        if not bool(msg.data):
            return
        self.get_logger().warn("[JOB] resume requested")

        # ✅ resume 요청이면 pause도 같이 해제 (resume implies unpause)
        if self._pause:
            self._pause = False
            self.get_logger().warn("[JOB] auto-unpause on resume")

        self._resume_requested = True

    # -----------------
    # tick (worker orchestration)
    # -----------------
    def tick(self):
        # 0) worker 종료 처리
        if self._running and self._worker_done:
            tok = self._worker_tok
            ok = self._worker_ok
            err = self._worker_err

            if ok and not self._stop_soft:
                self._publish_status(f"done:{tok}")
                self._set_status_step(self.STEP_DONE, "완료(done)")
                self._stopped = False
                self._checkpoint = None

            elif err == "stopped":
                ck = self._checkpoint_to_string()
                self._publish_status(f"stopped:{tok}:{ck}")
                self._set_status_step(self.STEP_IDLE, f"중단(stopped) ckpt={ck}")
                self._stopped = True
                self._last_token = tok

            else:
                if self._stop_soft and not err:
                    err = "aborted(stop_soft)"
                self._publish_status(f"error:{tok}:{err or 'aborted/failed'}")
                self._set_status_step(self.STEP_ERROR, f"에러(error) {err}")
                self._stopped = False
                self._checkpoint = None

            # running reset
            self._running = False
            self._worker = None
            self._worker_tok = None
            self._worker_done = True
            self._worker_ok = False
            self._worker_err = ""
            return

        # 1) worker가 돌고 있으면 return
        if self._running:
            return

        # 2) resume 처리
        if self._resume_requested:
            if self._stop_soft:
                self.get_logger().warn("[JOB] resume pending: stop_soft=True (waiting stop_soft False)")
                return  # 플래그 유지

            self._resume_requested = False

            if not self._stopped or self._last_token is None or self._checkpoint is None:
                self.get_logger().warn(
                    f"[JOB] resume ignored: not in stopped state (stopped={self._stopped}, "
                    f"last_token={self._last_token}, ckpt={self._checkpoint_to_string()})"
                )
                return

            tok = int(self._last_token)
            ckpt = dict(self._checkpoint)
            self.get_logger().warn(f"[JOB] resume start tok={tok} from ckpt={self._checkpoint_to_string()}")
            self._start_worker(tok=tok, start_ckpt=ckpt, resume_mode=True)
            return

        # 3) 새 작업 시작
        if self._pending_token is None:
            return

        tok = int(self._pending_token)
        self._pending_token = None

        if self._stop_soft:
            self.get_logger().warn("[JOB] stop_soft=True -> skip token")
            return

        # 새 작업이면 ckpt 초기화
        self._checkpoint = None
        self._stopped = False
        self._last_token = tok

        self._start_worker(tok=tok, start_ckpt=None, resume_mode=False)

    def _start_worker(self, tok: int, start_ckpt: Optional[Dict[str, Any]], resume_mode: bool):
        self._running = True
        self._worker_done = False
        self._worker_ok = False
        self._worker_tok = tok
        self._worker_err = ""

        def _run_worker():
            try:
                self._wait_if_paused()
                if self._stop_soft:
                    self._worker_ok = False
                    self._worker_err = "stopped"
                    return

                self._set_status_step(self.STEP_RUNNING, f"작업중(running) tok={tok} resume={resume_mode}")
                self.get_logger().info(f"[JOB] worker start token={tok} resume_mode={resume_mode}")

                ok = self._perform_cycle(start_ckpt=start_ckpt, resume_mode=resume_mode)
                self._worker_ok = bool(ok)

                if not ok and not self._worker_err:
                    self._worker_err = "stopped" if self._stop_soft else "aborted/failed"

            except Exception as e:
                if self._stop_soft:
                    self.get_logger().warn(f"[JOB] exception during stop -> treat as stopped: {e}")
                    self._worker_ok = False
                    self._worker_err = "stopped"
                else:
                    self.get_logger().error(f"[JOB] exception in worker: {e}")
                    self.get_logger().error(traceback.format_exc())
                    self._worker_ok = False
                    self._worker_err = str(e)

            finally:
                self._worker_done = True

        self._worker = threading.Thread(target=_run_worker, daemon=True)
        self._worker.start()

    # -----------------
    # job logic (override point)
    # -----------------
    def _perform_cycle(self, start_ckpt: Optional[Dict[str, Any]], resume_mode: bool) -> bool:
        """
        ✅ 여기에 작업 시퀀스를 작성.
        - checkpoint 업데이트는 self._set_ckpt({...}) 호출
        - stop_soft/pause 대응은:
            - 루프 중간중간 if self._check_abort(): self._worker_err="stopped"; return False
            - 긴 대기는 self._sleep_interruptible(sec) 사용
        - resume_mode=True 이면 start_ckpt부터 이어서 실행(restart from checkpoint)

        아래는 "형태"만 보여주는 예시.
        """
        ck = start_ckpt or self._checkpoint or {"phase": "PREPARE", "i": 0}

        # resume safe entry 같은 게 필요하면 여기서 처리
        if resume_mode:
            # 예: 홈정렬/툴세팅/상태정리 등
            self.get_logger().warn(f"[JOB] resume entry from ckpt={ck}")
            # 필요 시 ck phase 보수적으로 롤백
            # if ck["phase"] in ("SOME_RISKY_PHASE",): ck={"phase":"PREPARE","i":0}

        # PREPARE
        if ck.get("phase") == "PREPARE":
            self._set_ckpt({"phase": "PREPARE"})
            if self._check_abort():
                self._worker_err = "stopped"
                return False
            # 준비 동작...
            if not self._sleep_interruptible(0.2):
                self._worker_err = "stopped"
                return False
            ck = {"phase": "LOOP", "i": 0}
            self._set_ckpt(ck)

        # LOOP
        if ck.get("phase") == "LOOP":
            i0 = int(ck.get("i", 0))
            for i in range(i0, 5):
                self._set_ckpt({"phase": "LOOP", "i": i})
                if self._check_abort():
                    self._worker_err = "stopped"
                    return False

                # 실제 작업 1스텝...
                if not self._sleep_interruptible(0.5):
                    self._worker_err = "stopped"
                    return False

            ck = {"phase": "FINISH"}
            self._set_ckpt(ck)

        # FINISH
        if ck.get("phase") == "FINISH":
            self._set_ckpt({"phase": "FINISH"})
            if self._check_abort():
                self._worker_err = "stopped"
                return False
            # 마무리 동작...
            if not self._sleep_interruptible(0.2):
                self._worker_err = "stopped"
                return False

            self._set_ckpt({"phase": "DONE"})
            return True

        # DONE
        if ck.get("phase") == "DONE":
            return True

        # unknown phase
        self._worker_err = f"invalid_phase:{ck.get('phase')}"
        return False


def main(args=None):
    rclpy.init(args=args)
    cfg = RobotConfig()

    # boot node (DSR 내부에서 spin 돌린다는 전제 유지)
    boot = rclpy.create_node("dsr_boot_generic_job", namespace=cfg.robot_id)
    DR_init.__dsr__id = cfg.robot_id
    DR_init.__dsr__model = cfg.robot_model
    DR_init.__dsr__node = boot

    import DSR_ROBOT2  # noqa: F401

    # ✅ 여기서 노드 이름/토픽 세트만 바꾸면 "다른 작업 노드"로 재사용 가능
    node = GenericJobNode(
        cfg, boot,
        node_name="check_level_motion_node",     # <- 예시
        topic_run_once="/check_level/run_once",
        topic_resume="/check_level/resume",
        topic_status="/check_level/status",
        topic_step="/check_level/step",
        topic_state="/robot/state",
    )

    from rclpy.executors import SingleThreadedExecutor
    ex = SingleThreadedExecutor()
    ex.add_node(node)

    try:
        while rclpy.ok():
            ex.spin_once(timeout_sec=0.1)
            node.tick()
    finally:
        try:
            ex.remove_node(node)
        except Exception:
            pass
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