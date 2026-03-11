"""
firebase_bridge/action_handler.py
ExecuteJob 액션 클라이언트 처리:
  - goal 전송 / 응답 / 결과 / 피드백
  - 디자인 패턴 파싱
  - 협동작업(cowork) 상태 감지 및 스레드 분기
"""

import threading
import time

from tilemate_msgs.action import ExecuteJob
from .constants import DESIGN_PATTERNS, TILE_SYMBOL_MAP


class ActionHandlerMixin:
    """
    FirebaseBridgeNode에 믹스인으로 사용.
    self.ref, self.task_job_client, self._cement_waiting,
    self._cement_wait_thread, self._last_cowork_stage,
    self._last_completed_jobs_fb, self._last_tile_step_fb,
    self._active_goal_handle, self._active_result_future,
    self._active_token, self.get_logger() 에 접근 가정.
    """

    # ── 패턴 파싱 ────────────────────────────────────────
    @staticmethod
    def _pattern_to_layout(pattern_str: str) -> list:
        """
        "B,A,B,A,B,A,B,A,B" → [2,1,2,1,2,1,2,1,2]
        A=흰색=1, B=검정=2, C=데코=3
        """
        if not pattern_str:
            return []
        tokens = [x.strip().upper() for x in pattern_str.split(",") if x.strip()]
        layout = []
        for t in tokens:
            if t not in TILE_SYMBOL_MAP:
                raise ValueError(f"Unknown tile symbol: {t}")
            layout.append(TILE_SYMBOL_MAP[t])
        return layout

    @staticmethod
    def _resolve_design_pattern(cmd_dict: dict) -> tuple:
        """Firebase design 번호 → (pattern_str, design_id)"""
        design = int(cmd_dict.get("design", 0))
        if design in DESIGN_PATTERNS:
            return DESIGN_PATTERNS[design], design
        if design == 3:
            custom = str(cmd_dict.get("custom_pattern", "")).strip()
            if not custom:
                raise ValueError("design=3 but no custom_pattern")
            return custom, design
        raise ValueError(f"Unsupported design: {design}")

    # ── Goal 전송 ────────────────────────────────────────
    def _send_task_job_goal(self, cmd_dict: dict, is_resume: bool):
        if not self.task_job_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("ExecuteJob action server not available: /task/run_job")
            self.ref.update({"state": "작업 서버 연결 실패"})
            return

        pattern_str, design_id = self._resolve_design_pattern(cmd_dict)
        layout = self._pattern_to_layout(pattern_str)

        token             = str(cmd_dict.get("token", f"job_{int(time.time() * 1000)}"))
        completed_jobs    = int(cmd_dict.get("completed_jobs", 0))
        current_step      = int(cmd_dict.get("current_step", 0))
        current_tile_idx  = int(cmd_dict.get("current_tile_index", completed_jobs))

        goal = ExecuteJob.Goal()
        goal.token             = token
        goal.design_layout     = layout
        goal.is_resume         = bool(is_resume)
        goal.completed_jobs    = completed_jobs
        goal.current_step      = current_step
        goal.current_tile_index = current_tile_idx

        self._active_token           = token
        self._last_completed_jobs_fb = completed_jobs if is_resume else 0
        self._last_tile_step_fb      = current_step   if is_resume else 0

        self.ref.update({
            "state": "작업 시작 요청",
            "design": design_id,
            "design_ab": pattern_str,
            "token": token,
            "is_resume": bool(is_resume),
            "completed_jobs": completed_jobs if is_resume else 0,
            "current_step":   current_step   if is_resume else 0,
            "working_tile":   current_tile_idx if is_resume else 0,
            "overall_progress": 0.0,
            "tile_progress":    0.0,
            "tile_step": current_step if is_resume else 0,
            "message": "",
        })

        self.get_logger().info(
            f"[TASK_JOB] send goal: token={token}, resume={is_resume}, "
            f"layout={layout}, completed_jobs={completed_jobs}, "
            f"current_step={current_step}, current_tile_index={current_tile_idx}"
        )

        future = self.task_job_client.send_goal_async(
            goal, feedback_callback=self._fb_task_job
        )
        future.add_done_callback(self._on_task_job_goal_response)

    # ── 응답 콜백 ────────────────────────────────────────
    def _on_task_job_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"[TASK_JOB] goal response failed: {e}")
            self.ref.update({"state": f"goal response error: {e}"})
            return

        if not goal_handle.accepted:
            self.get_logger().error("[TASK_JOB] goal rejected")
            self.ref.update({"state": "작업 요청 거절"})
            self._active_goal_handle = None
            return

        self.get_logger().info("[TASK_JOB] goal accepted")
        self.ref.update({"state": "작업 수락됨"})
        self._active_goal_handle = goal_handle
        self._active_result_future = goal_handle.get_result_async()
        self._active_result_future.add_done_callback(self._on_task_job_result)

    # ── 결과 콜백 ────────────────────────────────────────
    def _on_task_job_result(self, future):
        try:
            result = future.result().result
        except Exception as e:
            self.get_logger().error(f"[TASK_JOB] result failed: {e}")
            self.ref.update({"state": f"result error: {e}"})
            self._active_goal_handle = None
            self._active_result_future = None
            return

        self._active_goal_handle = None
        self._active_result_future = None

        if result.success:
            self.get_logger().info(f"[TASK_JOB] success: {result.message}")
            self.ref.update({
                "state": "작업 완료",
                "message": result.message,
                "overall_progress": 1.0,
                "tile_progress": 1.0,
            })
        else:
            self.get_logger().warn(f"[TASK_JOB] failed: {result.message}")
            self.ref.update({"state": "작업 실패", "message": result.message})

    # ── 피드백 콜백 ──────────────────────────────────────
    def _fb_task_job(self, fb_msg):
        fb = fb_msg.feedback

        tile_index   = int(fb.tile_index)
        tile_type    = int(fb.tile_type)
        tile_step    = int(fb.detail_step)   # detail_step → tile_step 매핑
        overall_step = int(fb.overall_step)

        print(
            f"\n[FB] ─────────────────────────────────────\n"
            f"  overall_step   : {overall_step}\n"
            f"  tile_step      : {tile_step}  (prev={self._last_tile_step_fb})\n"
            f"  tile_index     : {tile_index}\n"
            f"  tile_type      : {tile_type}\n"
            f"  overall_prog   : {float(fb.overall_progress):.3f}\n"
            f"  detail_prog    : {float(fb.detail_progress):.3f}\n"
            f"  state          : {fb.state}\n"
            f"─────────────────────────────────────────",
            flush=True,
        )

        # 타일 완료 개수 추정 (tile_step == 5 = TILE_STEP_DONE)
        completed_jobs = self._last_completed_jobs_fb
        if tile_step == 5 and self._last_tile_step_fb != 5:
            completed_jobs = max(completed_jobs, tile_index + 1)
            self._last_completed_jobs_fb = completed_jobs

        current_state = str(fb.state)
        self._handle_cowork_state(current_state)

        self._last_cowork_stage = current_state
        self._last_tile_step_fb = tile_step

        self.ref.update({
            "current_step":     overall_step,
            "overall_progress": float(fb.overall_progress),
            "working_tile":     tile_index,
            "tile_type":        tile_type,
            "tile_step":        tile_step,
            "tile_progress":    float(fb.detail_progress),  # detail_progress → tile_progress
            "completed_jobs":   completed_jobs,
            "state":            current_state,
        })

    def _handle_cowork_state(self, current_state: str):
        """WAIT_HUMAN_TAKE / WAIT_CEMENT_DONE 감지 후 스레드 분기."""
        if (
            "WAIT_HUMAN_TAKE" in current_state
            and "WAIT_HUMAN_TAKE" not in self._last_cowork_stage
            and not self._cement_waiting
        ):
            self.get_logger().info(f"[CEMENT] WAIT_HUMAN_TAKE 감지 → waiting_pick 시작")
            self._cement_waiting = True
            self._cement_wait_thread = threading.Thread(
                target=self._cement_pick_flow, daemon=True
            )
            self._cement_wait_thread.start()

        elif (
            "WAIT_CEMENT_DONE" in current_state
            and "WAIT_CEMENT_DONE" not in self._last_cowork_stage
            and not self._cement_waiting
        ):
            self.get_logger().info(f"[CEMENT] WAIT_CEMENT_DONE 감지 → waiting_cement 시작")
            self._cement_waiting = True
            self._cement_wait_thread = threading.Thread(
                target=self._cement_done_flow, daemon=True
            )
            self._cement_wait_thread.start()

    # ── 취소 ────────────────────────────────────────────
    def _cancel_active_goal(self):
        if self._active_goal_handle is None:
            self.get_logger().warn("[TASK_JOB] no active goal to cancel")
            return
        try:
            self.get_logger().warn("[TASK_JOB] cancel active goal")
            self._active_goal_handle.cancel_goal_async().add_done_callback(
                self._on_cancel_done
            )
        except Exception as e:
            self.get_logger().error(f"[TASK_JOB] cancel failed: {e}")

    def _on_cancel_done(self, future):
        try:
            future.result()
            self.get_logger().warn("[TASK_JOB] cancel response received")
            self.ref.update({"state": "작업 취소 요청됨"})
        except Exception as e:
            self.get_logger().error(f"[TASK_JOB] cancel result error: {e}")
