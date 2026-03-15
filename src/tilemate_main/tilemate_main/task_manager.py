#!/usr/bin/env python3
import time
import traceback
from typing import Callable, Tuple, Any, Awaitable
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String
from std_srvs.srv import Trigger
from action_msgs.msg import GoalStatus

from tilemate_msgs.action import ExecuteJob, PickTile, PlaceTile, Cowork, Inspect, Press, PatternInspect
from tilemate_main.robot_config import RobotConfig


@dataclass
class TileRunContext:
    goal_handle: Any
    tile_index: int
    tile_type: int
    total_tiles: int


@dataclass
class TileStepDef:
    step_id: int
    name: str
    runner: Callable[[TileRunContext], Awaitable[tuple[bool, str]]]


class TaskManagerNode(Node):

    OVERALL_READY = 0
    OVERALL_TILE_WORK = 1
    OVERALL_FINISHED = 2

    TILE_STEP_IDLE = 0
    TILE_STEP_PICK = 1
    TILE_STEP_COWORK = 2
    TILE_STEP_PLACE = 3
    TILE_STEP_INSPECT = 4
    TILE_STEP_COMPACT = 5
    TILE_STEP_DONE = 6

    def __init__(self):
        super().__init__("task_manager_node")
        self.cb_group = ReentrantCallbackGroup()

        self.active_job_goal = None
        self.active_sub_goal = None
        self.kill_requested = False

        self._action_server = ActionServer(
            self,
            ExecuteJob,
            "task/run_job",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group,
        )

        self._kill_srv = self.create_service(
            Trigger,
            "/kill",
            self.kill_service_callback,
            callback_group=self.cb_group,
        )

        self.pub_robot_cmd = self.create_publisher(String, "/robot/command", 10)

        robot_ns = f"/{RobotConfig.robot_id}"

        self.pick_client = ActionClient(
            self, PickTile, f"{robot_ns}/tile/pick", callback_group=self.cb_group
        )
        self.cowork_client = ActionClient(
            self, Cowork, f"{robot_ns}/tile/cowork", callback_group=self.cb_group
        )
        self.place_client = ActionClient(
            self, PlaceTile, f"{robot_ns}/tile/place_press", callback_group=self.cb_group
        )
        self.inspect_client = ActionClient(
            self, Inspect, f"{robot_ns}/tile/inspect", callback_group=self.cb_group
        )
        self.press_client = ActionClient(
            self, Press, f"{robot_ns}/tile/press", callback_group=self.cb_group
        )
        self.pattern_inspect_client = ActionClient(
            self, PatternInspect, f"{robot_ns}/tile/pattern_inspect", callback_group=self.cb_group
        )

        self._current_total_tiles = 0
        self.current_tile_index = -1
        self.current_tile_type = -1
        self.current_detail_step = self.TILE_STEP_IDLE
        self.current_detail_progress = 0.0
        self.current_state = ""

        self.tile_steps = [
            TileStepDef(self.TILE_STEP_PICK, "pick", self._step_pick),
            TileStepDef(self.TILE_STEP_COWORK, "cowork", self._step_cowork),
            TileStepDef(self.TILE_STEP_PLACE, "place", self._step_place),
            # 9개 다 설치하고 검사할 경우 주석
            # TileStepDef(self.TILE_STEP_INSPECT, "inspect", self._step_inspect),
            # TileStepDef(self.TILE_STEP_COMPACT, "compact", self._step_compact),
        ]

        self.get_logger().info(
            f"name={self.get_name()}, ns={self.get_namespace()}, fq={self.get_fully_qualified_name()}"
        )
        self.get_logger().info("\033[94m [1/6] [TASK_MANAGER] initialize Done!\033[0m")

    # --------------------------------------------------
    # common state / feedback
    # --------------------------------------------------
    def _update_and_publish_feedback(
        self,
        goal_handle,
        tile_index: int,
        tile_type: int,
        detail_step: int,
        detail_progress: float,
        state: str,
        overall_step: int = None,
    ):
        if overall_step is None:
            overall_step = self.OVERALL_TILE_WORK

        overall_progress = self._calc_overall_progress(
            tile_index=tile_index,
            total_tiles=self._current_total_tiles,
            tile_step=detail_step,
            detail_progress=detail_progress,
        )

        self.current_tile_index = tile_index
        self.current_tile_type = tile_type
        self.current_detail_step = detail_step
        self.current_detail_progress = detail_progress
        self.current_state = state

        fb = ExecuteJob.Feedback()
        fb.overall_step = int(overall_step)
        fb.overall_progress = float(overall_progress)
        fb.tile_index = int(tile_index)
        fb.tile_type = int(tile_type)
        fb.detail_step = int(detail_step)
        fb.detail_progress = float(detail_progress)
        fb.state = str(state)
        goal_handle.publish_feedback(fb)

    def _calc_overall_progress(
        self,
        tile_index: int,
        total_tiles: int,
        tile_step: int,
        detail_progress: float,
    ) -> float:
        if total_tiles <= 0:
            return 0.0

        ordered_step_ids = [step.step_id for step in self.tile_steps]
        units_per_tile = float(len(ordered_step_ids))

        total_units = float(total_tiles) * units_per_tile
        done_units = float(tile_index) * units_per_tile

        if tile_step >= self.TILE_STEP_DONE:
            done_units += units_per_tile
        else:
            try:
                step_order = ordered_step_ids.index(tile_step)
                done_units += float(step_order) + float(detail_progress)
            except ValueError:
                pass

        p = done_units / total_units
        return max(0.0, min(1.0, p))

    # --------------------------------------------------
    # helpers
    # --------------------------------------------------
    def _publish_stop(self, repeat: int = 3, interval: float = 0.05):
        msg = String()
        msg.data = "stop"
        for i in range(repeat):
            self.pub_robot_cmd.publish(msg)
            if i < repeat - 1:
                time.sleep(interval)

    def _cancel_active_subgoal(self):
        if self.active_sub_goal is None:
            self.get_logger().warn("[KILL] no active sub goal")
            return

        try:
            fut = self.active_sub_goal.cancel_goal_async()
            fut.add_done_callback(self._on_cancel_subgoal_done)
        except Exception as e:
            self.get_logger().error(f"[KILL] sub goal cancel failed: {e}")

    def _on_cancel_subgoal_done(self, future):
        try:
            future.result()
            self.get_logger().warn("[KILL] sub goal cancel response received")
        except Exception as e:
            self.get_logger().error(f"[KILL] sub goal cancel response error: {e}")

    def _check_abort_requested(self, goal_handle, phase: str) -> Tuple[bool, str]:
        if self.kill_requested:
            return True, f"killed_{phase}"
        if goal_handle.is_cancel_requested:
            return True, f"canceled_{phase}"
        return False, ""

    def _is_tile_absent_message(self, msg: str) -> bool:
        if not msg:
            return False

        s = str(msg).lower()
        keywords = [
            "tile_not_found",
            "tray_empty",
            "depth_target_not_found",
            "depth_value_missing",
        ]
        return any(k in s for k in keywords)

    # --------------------------------------------------
    # /kill
    # --------------------------------------------------
    def kill_service_callback(self, request, response):
        del request
        self.kill_requested = True
        self._cancel_active_subgoal()
        self._publish_stop()
        response.success = True
        response.message = "kill requested"
        return response

    # --------------------------------------------------
    # goal / cancel
    # --------------------------------------------------
    def goal_callback(self, goal_request):
        n = len(goal_request.design_layout)

        if n <= 0:
            return GoalResponse.REJECT
        if goal_request.completed_jobs < 0:
            return GoalResponse.REJECT
        if goal_request.current_tile_index < 0:
            return GoalResponse.REJECT
        if goal_request.current_tile_index > n:
            return GoalResponse.REJECT
        if goal_request.completed_jobs > n:
            return GoalResponse.REJECT

        self.get_logger().info(
            f"Goal accepted: token={goal_request.token}, "
            f"tiles={n}, resume={goal_request.is_resume}, "
            f"completed_jobs={goal_request.completed_jobs}, "
            f"current_step={goal_request.current_step}, "
            f"current_tile_index={goal_request.current_tile_index}"
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        del goal_handle
        self.kill_requested = True
        self._cancel_active_subgoal()
        self._publish_stop()
        return CancelResponse.ACCEPT

    # --------------------------------------------------
    # feedback adapters
    # --------------------------------------------------
    def _make_pick_feedback_cb(self, goal_handle, tile_index: int, tile_type: int):
        def _cb(feedback_msg):
            fb = feedback_msg.feedback
            progress = float(getattr(fb, "progress", 0.0))
            state = str(getattr(fb, "state", "pick"))
            self._update_and_publish_feedback(
                goal_handle=goal_handle,
                tile_index=tile_index,
                tile_type=tile_type,
                detail_step=self.TILE_STEP_PICK,
                detail_progress=progress,
                state=f"pick:{state}",
            )
        return _cb

    def _make_cowork_feedback_cb(self, goal_handle, tile_index: int, tile_type: int):
        def _cb(feedback_msg):
            fb = feedback_msg.feedback
            current_step = int(getattr(fb, "current_step", 0))
            stage = str(getattr(fb, "stage", "cowork"))
            detail = str(getattr(fb, "detail", ""))

            progress = max(0.0, min(1.0, float(current_step) / 11.0))

            self._update_and_publish_feedback(
                goal_handle=goal_handle,
                tile_index=tile_index,
                tile_type=tile_type,
                detail_step=self.TILE_STEP_COWORK,
                detail_progress=progress,
                state=f"cowork:{stage}:{detail}",
            )
        return _cb

    def _make_place_feedback_cb(self, goal_handle, tile_index: int, tile_type: int):
        def _cb(feedback_msg):
            fb = feedback_msg.feedback
            progress = float(getattr(fb, "progress", 0.0))
            state = str(getattr(fb, "state", "place"))
            self._update_and_publish_feedback(
                goal_handle=goal_handle,
                tile_index=tile_index,
                tile_type=tile_type,
                detail_step=self.TILE_STEP_PLACE,
                detail_progress=progress,
                state=f"place:{state}",
            )
        return _cb

    def _make_inspect_feedback_cb(self, goal_handle, tile_index: int, tile_type: int):
        def _cb(feedback_msg):
            fb = feedback_msg.feedback
            step = int(getattr(fb, "step", 0))
            progress = float(getattr(fb, "progress", 0.0))
            state = str(getattr(fb, "state", "inspect"))

            if progress > 1.0:
                progress = progress / 100.0

            progress = max(0.0, min(1.0, progress))

            self._update_and_publish_feedback(
                goal_handle=goal_handle,
                tile_index=tile_index,
                tile_type=tile_type,
                detail_step=self.TILE_STEP_INSPECT,
                detail_progress=progress,
                state=f"inspect:{step}:{state}",
            )
        return _cb

    def _make_press_feedback_cb(self, goal_handle, tile_index: int, tile_type: int):
        def _cb(feedback_msg):
            fb = feedback_msg.feedback
            step = int(getattr(fb, "step", 0))
            progress = float(getattr(fb, "progress", 0.0))
            state = str(getattr(fb, "state", "compact"))

            if progress > 1.0:
                progress = progress / 100.0

            progress = max(0.0, min(1.0, progress))

            self._update_and_publish_feedback(
                goal_handle=goal_handle,
                tile_index=tile_index,
                tile_type=tile_type,
                detail_step=self.TILE_STEP_COMPACT,
                detail_progress=progress,
                state=f"compact:{step}:{state}",
            )
        return _cb

    def _make_pattern_inspect_feedback_cb(self, goal_handle, tile_index: int, tile_type: int):
        def _cb(feedback_msg):
            fb = feedback_msg.feedback
            current_frame = int(getattr(fb, "current_frame", 0))
            progress = float(getattr(fb, "progress", 0.0))
            state = str(getattr(fb, "state", "pattern_inspect"))

            self._update_and_publish_feedback(
                goal_handle=goal_handle,
                tile_index=tile_index,
                tile_type=tile_type,
                detail_step=self.TILE_STEP_INSPECT,
                detail_progress=progress,
                state=f"inspect:pattern:{current_frame}:{state}",
            )
        return _cb

    # --------------------------------------------------
    # generic action runner
    # --------------------------------------------------
    async def _run_action_subtask(
        self,
        *,
        client,
        client_name: str,
        goal_msg,
        feedback_callback,
        goal_handle,
        phase_name: str,
    ) -> Tuple[bool, str]:
        if self.kill_requested:
            return False, f"killed_before_{phase_name}"

        if not client.wait_for_server(timeout_sec=2.0):
            return False, f"{client_name}_server_unavailable"

        send_future = client.send_goal_async(
            goal_msg,
            feedback_callback=feedback_callback,
        )
        sub_goal_handle = await send_future
        self.active_sub_goal = sub_goal_handle

        if not sub_goal_handle.accepted:
            self.active_sub_goal = None
            return False, f"{phase_name}_goal_rejected"

        aborted, msg = self._check_abort_requested(goal_handle, f"during_{phase_name}_start")
        if aborted:
            try:
                await sub_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self.active_sub_goal = None
            return False, msg

        result_wrap = await sub_goal_handle.get_result_async()
        self.active_sub_goal = None

        if result_wrap.status == GoalStatus.STATUS_CANCELED:
            if self.kill_requested:
                return False, f"{phase_name}_canceled_by_kill"
            return False, f"{phase_name}_canceled"

        if result_wrap.status != GoalStatus.STATUS_SUCCEEDED:
            return False, f"{phase_name}_failed_status_{result_wrap.status}"

        result = result_wrap.result
        if not result.success:
            return False, result.message

        return True, result.message

    # --------------------------------------------------
    # specific subtasks
    # --------------------------------------------------
    async def _step_pick(self, ctx: TileRunContext):
        return await self.call_pick_tile(
            goal_handle=ctx.goal_handle,
            tile_index=ctx.tile_index,
            tile_type=ctx.tile_type,
        )

    async def _step_cowork(self, ctx: TileRunContext):
        return await self.call_cowork(
            goal_handle=ctx.goal_handle,
            tile_index=ctx.tile_index,
            tile_type=ctx.tile_type,
            wait_cement_done=True,
        )

    async def _step_place(self, ctx: TileRunContext):
        return await self.call_place_tile(
            goal_handle=ctx.goal_handle,
            tile_index=ctx.tile_index,
            tile_type=ctx.tile_type,
        )

    async def _step_inspect(self, ctx: TileRunContext):
        ok, msg = await self.call_inspect(
            goal_handle=ctx.goal_handle,
            tile_index=ctx.tile_index,
            tile_type=ctx.tile_type,
            total_tiles=ctx.total_tiles,
        )
        if not ok:
            return False, f"depth_inspect_failed:{msg}"

        ok, msg = await self.call_pattern_inspect(
            goal_handle=ctx.goal_handle,
            tile_index=ctx.tile_index,
            tile_type=ctx.tile_type,
        )
        if not ok:
            return False, f"pattern_inspect_failed:{msg}"

        return True, "inspect_all_ok"

    async def _step_compact(self, ctx: TileRunContext):
        return await self.call_press(
            goal_handle=ctx.goal_handle,
            tile_index=ctx.tile_index,
            tile_type=ctx.tile_type,
        )

    async def call_pick_tile(self, goal_handle, tile_index, tile_type):
        goal = PickTile.Goal()
        goal.tile_index = tile_index + 1
        goal.tile_type = tile_type

        return await self._run_action_subtask(
            client=self.pick_client,
            client_name="pick_action",
            goal_msg=goal,
            feedback_callback=self._make_pick_feedback_cb(goal_handle, tile_index, tile_type),
            goal_handle=goal_handle,
            phase_name="pick",
        )

    async def call_cowork(self, goal_handle, tile_index, tile_type, wait_cement_done=True):
        goal = Cowork.Goal()
        goal.tile_index = tile_index + 1
        goal.tile_type = int(tile_type)
        goal.wait_cement_done = bool(wait_cement_done)

        return await self._run_action_subtask(
            client=self.cowork_client,
            client_name="cowork_action",
            goal_msg=goal,
            feedback_callback=self._make_cowork_feedback_cb(goal_handle, tile_index, tile_type),
            goal_handle=goal_handle,
            phase_name="cowork",
        )

    async def call_place_tile(self, goal_handle, tile_index, tile_type):
        goal = PlaceTile.Goal()
        goal.placement_index = 1
        goal.search_force = 15.0
        goal.press_force = 40.0
        goal.hold_time = 0.8
        goal.target_press_depth = 3.0

        return await self._run_action_subtask(
            client=self.place_client,
            client_name="place_action",
            goal_msg=goal,
            feedback_callback=self._make_place_feedback_cb(goal_handle, tile_index, tile_type),
            goal_handle=goal_handle,
            phase_name="place",
        )

    async def call_inspect(self, goal_handle, tile_index, tile_type, total_tiles):
        del total_tiles

        goal = Inspect.Goal()

        self._update_and_publish_feedback(
            goal_handle=goal_handle,
            tile_index=tile_index,
            tile_type=tile_type,
            detail_step=self.TILE_STEP_INSPECT,
            detail_progress=0.0,
            state="inspect:goal_send",
        )

        return await self._run_action_subtask(
            client=self.inspect_client,
            client_name="inspect_action",
            goal_msg=goal,
            feedback_callback=self._make_inspect_feedback_cb(goal_handle, tile_index, tile_type),
            goal_handle=goal_handle,
            phase_name="inspect",
        )

    async def call_pattern_inspect(self, goal_handle, tile_index, tile_type):
        goal = PatternInspect.Goal()
        goal.target_frames = 60
        goal.pixel_threshold = 60
        goal.match_dist_threshold = 50.0

        return await self._run_action_subtask(
            client=self.pattern_inspect_client,
            client_name="pattern_inspect_action",
            goal_msg=goal,
            feedback_callback=self._make_pattern_inspect_feedback_cb(goal_handle, tile_index, tile_type),
            goal_handle=goal_handle,
            phase_name="pattern_inspect",
        )

    async def call_press(self, goal_handle, tile_index, tile_type):
        goal = Press.Goal()
        goal.result_json_path = ""
        goal.press_threshold_mm = 1.0
        goal.approach_offset_mm = 20.0
        goal.press_overshoot_mm = 1.0
        goal.press_force_n = 30.0
        goal.press_speed_mm_s = 30.0

        self._update_and_publish_feedback(
            goal_handle=goal_handle,
            tile_index=tile_index,
            tile_type=tile_type,
            detail_step=self.TILE_STEP_COMPACT,
            detail_progress=0.0,
            state="compact:goal_send",
        )

        return await self._run_action_subtask(
            client=self.press_client,
            client_name="press_action",
            goal_msg=goal,
            feedback_callback=self._make_press_feedback_cb(goal_handle, tile_index, tile_type),
            goal_handle=goal_handle,
            phase_name="compact",
        )

    # --------------------------------------------------
    # execute
    # --------------------------------------------------

    # 타일 한개마다 검사 모드
    # async def execute_callback(self, goal_handle):
    #     result = ExecuteJob.Result()
    #     req = goal_handle.request
    #
    #     self.active_job_goal = goal_handle
    #     self.active_sub_goal = None
    #     self.kill_requested = False
    #
    #     try:
    #         layout = list(req.design_layout)
    #         total_tiles = len(layout)
    #         self._current_total_tiles = total_tiles
    #
    #         start_tile_index = int(req.current_tile_index) if req.is_resume else 0
    #         start_step = int(req.current_step) if req.is_resume else self.TILE_STEP_IDLE
    #
    #         if start_tile_index >= total_tiles:
    #             goal_handle.succeed()
    #             result.success = True
    #             result.message = "already_finished"
    #             return result
    #
    #         for tile_index in range(start_tile_index, total_tiles):
    #             aborted, msg = self._check_abort_requested(goal_handle, "before_tile_start")
    #             if aborted:
    #                 if self.kill_requested:
    #                     goal_handle.abort()
    #                     result.success = False
    #                     result.message = msg
    #                 else:
    #                     goal_handle.canceled()
    #                     result.success = False
    #                     result.message = msg
    #                 return result
    #
    #             tile_type = int(layout[tile_index])
    #             tile_step_start = int(start_step) if (req.is_resume and tile_index == start_tile_index) else self.TILE_STEP_IDLE
    #
    #             ok, msg = await self.run_single_tile(
    #                 goal_handle=goal_handle,
    #                 tile_index=tile_index,
    #                 tile_type=tile_type,
    #                 total_tiles=total_tiles,
    #                 tile_step_start=tile_step_start,
    #             )
    #
    #             if not ok:
    #                 if self.kill_requested:
    #                     goal_handle.abort()
    #                     result.success = False
    #                     result.message = f"killed:{msg}"
    #                 elif goal_handle.is_cancel_requested:
    #                     goal_handle.canceled()
    #                     result.success = False
    #                     result.message = f"canceled:{msg}"
    #                 else:
    #                     goal_handle.abort()
    #                     result.success = False
    #                     result.message = msg
    #                 return result
    #
    #         self._update_and_publish_feedback(
    #             goal_handle=goal_handle,
    #             tile_index=total_tiles - 1 if total_tiles > 0 else 0,
    #             tile_type=int(layout[-1]) if total_tiles > 0 else 0,
    #             detail_step=self.TILE_STEP_DONE,
    #             detail_progress=1.0,
    #             state="done",
    #             overall_step=self.OVERALL_FINISHED,
    #         )
    #
    #         goal_handle.succeed()
    #         result.success = True
    #         result.message = "all_tiles_completed"
    #         return result
    #
    #     except Exception as e:
    #         self.get_logger().error(f"[TASK] execute failed: {repr(e)}")
    #         self.get_logger().error(traceback.format_exc())
    #         try:
    #             goal_handle.abort()
    #         except Exception:
    #             pass
    #         result.success = False
    #         result.message = f"exception:{repr(e)}"
    #         return result
    #
    #     finally:
    #         self.active_job_goal = None
    #         self.active_sub_goal = None
    #         self.kill_requested = False
    #         self._current_total_tiles = 0
    #         self.current_tile_index = -1
    #         self.current_tile_type = -1
    #         self.current_detail_step = self.TILE_STEP_IDLE
    #         self.current_detail_progress = 0.0
    #         self.current_state = ""

    # 타일 9개 설치하고 검사 모드
    async def execute_callback(self, goal_handle):
        result = ExecuteJob.Result()
        req = goal_handle.request

        self.active_job_goal = goal_handle
        self.active_sub_goal = None
        self.kill_requested = False

        try:
            layout = list(req.design_layout)
            total_tiles = len(layout)
            self._current_total_tiles = total_tiles

            start_tile_index = int(req.current_tile_index) if req.is_resume else 0
            start_step = int(req.current_step) if req.is_resume else self.TILE_STEP_IDLE

            if start_tile_index >= total_tiles:
                goal_handle.succeed()
                result.success = True
                result.message = "already_finished"
                return result

            # --------------------------------------------------
            # 1) 모든 타일에 대해 pick -> cowork -> place 수행
            # --------------------------------------------------
            for tile_index in range(start_tile_index, total_tiles):
                aborted, msg = self._check_abort_requested(goal_handle, "before_tile_start")
                if aborted:
                    if self.kill_requested:
                        goal_handle.abort()
                        result.success = False
                        result.message = msg
                    else:
                        goal_handle.canceled()
                        result.success = False
                        result.message = msg
                    return result

                tile_type = int(layout[tile_index])

                # resume일 때 시작 타일만 current_step부터 재개
                # 단, 타일 반복 단계는 pick/cowork/place까지만 허용
                if req.is_resume and tile_index == start_tile_index:
                    tile_step_start = int(start_step)
                    if tile_step_start > self.TILE_STEP_PLACE:
                        tile_step_start = self.TILE_STEP_IDLE
                else:
                    tile_step_start = self.TILE_STEP_IDLE

                self.get_logger().info(
                    f"[TASK] tile start: index={tile_index}, type={tile_type}, "
                    f"tile_step_start={tile_step_start}"
                )

                ok, msg = await self.run_single_tile(
                    goal_handle=goal_handle,
                    tile_index=tile_index,
                    tile_type=tile_type,
                    total_tiles=total_tiles,
                    tile_step_start=tile_step_start,
                )

                self.get_logger().info(
                    f"[TASK] tile done: index={tile_index}, ok={ok}, msg={msg}"
                )

                if not ok:
                    if self.kill_requested:
                        goal_handle.abort()
                        result.success = False
                        result.message = f"killed:{msg}"
                    elif goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        result.success = False
                        result.message = f"canceled:{msg}"
                    else:
                        goal_handle.abort()
                        result.success = False
                        result.message = msg
                    return result

            # --------------------------------------------------
            # 2) 전체 배치 완료 후 inspect 1회
            # --------------------------------------------------
            final_tile_index = total_tiles - 1
            final_tile_type = int(layout[-1]) if total_tiles > 0 else 0

            aborted, msg = self._check_abort_requested(goal_handle, "before_final_inspect")
            if aborted:
                if self.kill_requested:
                    goal_handle.abort()
                    result.success = False
                    result.message = msg
                else:
                    goal_handle.canceled()
                    result.success = False
                    result.message = msg
                return result

            self.get_logger().info("[TASK] final inspect start")

            ok, msg = await self.call_inspect(
                goal_handle=goal_handle,
                tile_index=final_tile_index,
                tile_type=final_tile_type,
                total_tiles=total_tiles,
            )

            self.get_logger().info(f"[TASK] final inspect done: ok={ok}, msg={msg}")

            if not ok:
                if self.kill_requested:
                    goal_handle.abort()
                    result.success = False
                    result.message = f"killed:inspect_failed:{msg}"
                elif goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = f"canceled:inspect_failed:{msg}"
                else:
                    goal_handle.abort()
                    result.success = False
                    result.message = f"inspect_failed:{msg}"
                return result

            # --------------------------------------------------
            # 3) 전체 배치 완료 후 pattern inspect 1회
            # --------------------------------------------------
            aborted, msg = self._check_abort_requested(goal_handle, "before_final_pattern_inspect")
            if aborted:
                if self.kill_requested:
                    goal_handle.abort()
                    result.success = False
                    result.message = msg
                else:
                    goal_handle.canceled()
                    result.success = False
                    result.message = msg
                return result

            self.get_logger().info("[TASK] final pattern inspect start")

            ok, msg = await self.call_pattern_inspect(
                goal_handle=goal_handle,
                tile_index=final_tile_index,
                tile_type=final_tile_type,
            )

            self.get_logger().info(f"[TASK] final pattern inspect done: ok={ok}, msg={msg}")

            if not ok:
                if self.kill_requested:
                    goal_handle.abort()
                    result.success = False
                    result.message = f"killed:pattern_inspect_failed:{msg}"
                elif goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = f"canceled:pattern_inspect_failed:{msg}"
                else:
                    goal_handle.abort()
                    result.success = False
                    result.message = f"pattern_inspect_failed:{msg}"
                return result

            # --------------------------------------------------
            # 4) 전체 검사 후 press 1회
            # --------------------------------------------------
            aborted, msg = self._check_abort_requested(goal_handle, "before_final_compact")
            if aborted:
                if self.kill_requested:
                    goal_handle.abort()
                    result.success = False
                    result.message = msg
                else:
                    goal_handle.canceled()
                    result.success = False
                    result.message = msg
                return result

            self.get_logger().info("[TASK] final compact start")

            ok, msg = await self.call_press(
                goal_handle=goal_handle,
                tile_index=final_tile_index,
                tile_type=final_tile_type,
            )

            self.get_logger().info(f"[TASK] final compact done: ok={ok}, msg={msg}")

            if not ok:
                if self.kill_requested:
                    goal_handle.abort()
                    result.success = False
                    result.message = f"killed:compact_failed:{msg}"
                elif goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = f"canceled:compact_failed:{msg}"
                else:
                    goal_handle.abort()
                    result.success = False
                    result.message = f"compact_failed:{msg}"
                return result

            # --------------------------------------------------
            # 5) 전체 완료
            # --------------------------------------------------
            self._update_and_publish_feedback(
                goal_handle=goal_handle,
                tile_index=final_tile_index if total_tiles > 0 else 0,
                tile_type=final_tile_type,
                detail_step=self.TILE_STEP_DONE,
                detail_progress=1.0,
                state="done",
                overall_step=self.OVERALL_FINISHED,
            )

            goal_handle.succeed()
            result.success = True
            result.message = "all_tiles_completed"
            return result

        except Exception as e:
            self.get_logger().error(f"[TASK] execute failed: {repr(e)}")
            self.get_logger().error(traceback.format_exc())
            try:
                goal_handle.abort()
            except Exception:
                pass
            result.success = False
            result.message = f"exception:{repr(e)}"
            return result

        finally:
            self.active_job_goal = None
            self.active_sub_goal = None
            self.kill_requested = False
            self._current_total_tiles = 0
            self.current_tile_index = -1
            self.current_tile_type = -1
            self.current_detail_step = self.TILE_STEP_IDLE
            self.current_detail_progress = 0.0
            self.current_state = ""

    async def run_single_tile(
        self,
        goal_handle,
        tile_index,
        tile_type,
        total_tiles,
        tile_step_start,
    ):
        ctx = TileRunContext(
            goal_handle=goal_handle,
            tile_index=tile_index,
            tile_type=tile_type,
            total_tiles=total_tiles,
        )

        for step_def in self.tile_steps:
            if tile_step_start > step_def.step_id:
                continue

            ok, msg = await step_def.runner(ctx)
            if not ok:
                fail_msg = f"{step_def.name}_failed:{msg}"

                # pick 단계에서 타일 없음 상태를 별도로 구분
                if step_def.step_id == self.TILE_STEP_PICK and self._is_tile_absent_message(msg):
                    self._update_and_publish_feedback(
                        goal_handle=goal_handle,
                        tile_index=tile_index,
                        tile_type=tile_type,
                        detail_step=self.TILE_STEP_PICK,
                        detail_progress=1.0,
                        state=f"pick:tile_absent:{msg}",
                    )
                    return False, f"tile_absent:{fail_msg}"

                # 일반 실패 상태
                self._update_and_publish_feedback(
                    goal_handle=goal_handle,
                    tile_index=tile_index,
                    tile_type=tile_type,
                    detail_step=step_def.step_id,
                    detail_progress=1.0,
                    state=f"{step_def.name}:failed:{msg}",
                )
                return False, fail_msg

            aborted, abort_msg = self._check_abort_requested(
                goal_handle, f"after_{step_def.name}"
            )
            if aborted:
                return False, abort_msg

        self._update_and_publish_feedback(
            goal_handle=goal_handle,
            tile_index=tile_index,
            tile_type=tile_type,
            detail_step=self.TILE_STEP_DONE,
            detail_progress=1.0,
            state="tile_done",
        )

        return True, "ok"


def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()
    ex = MultiThreadedExecutor()
    ex.add_node(node)

    try:
        ex.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted")
    finally:
        try:
            ex.remove_node(node)
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()