#!/usr/bin/env python3
import time
import traceback

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String
from std_srvs.srv import Trigger
from action_msgs.msg import GoalStatus

from tilemate_msgs.srv import Inspect
from tilemate_msgs.action import ExecuteJob, PickTile, PlaceTile
from tilemate_main.robot_config import RobotConfig


class TaskManagerNode(Node):

    # ----------------------------
    # overall step enum
    # ----------------------------
    OVERALL_READY = 0
    OVERALL_TILE_WORK = 1
    OVERALL_FINISHED = 2

    # ----------------------------
    # tile step enum
    # ----------------------------
    TILE_STEP_IDLE = 0
    TILE_STEP_PICK = 1
    TILE_STEP_PLACE = 2
    TILE_STEP_INSPECT = 3
    TILE_STEP_COMPACT = 4
    TILE_STEP_DONE = 5

    def __init__(self):
        super().__init__("task_manager_node")

        self.cb_group = ReentrantCallbackGroup()

        # ----------------------------
        # runtime state
        # ----------------------------
        self.active_job_goal = None
        self.active_sub_goal = None
        self.kill_requested = False

        # ----------------------------
        # 상위 작업 액션 서버
        # ----------------------------
        self._action_server = ActionServer(
            self,
            ExecuteJob,
            "task/run_job",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group,
        )

        # ----------------------------
        # 디버그/긴급 정지 서비스
        # ----------------------------
        self._kill_srv = self.create_service(
            Trigger,
            "/kill",
            self.kill_service_callback,
            callback_group=self.cb_group,
        )

        # ----------------------------
        # stop 호환용 토픽
        # ----------------------------
        self.pub_robot_cmd = self.create_publisher(String, "/robot/command", 10)

        # ----------------------------
        # 하위 action / service client
        # ----------------------------
        robot_ns = f"/{RobotConfig.robot_id}"

        self.pick_client = ActionClient(
            self,
            PickTile,
            f"{robot_ns}/tile/pick",
            callback_group=self.cb_group,
        )
        self.place_client = ActionClient(
            self,
            PlaceTile,
            f"{robot_ns}/tile/place_press",
            callback_group=self.cb_group,
        )
        self.inspect_client = self.create_client(
            Inspect,
            f"{robot_ns}/tile/inspect",
            callback_group=self.cb_group,
        )

        # ----------------------------
        # state cache
        # ----------------------------
        self._current_total_tiles = 0
        self.current_tile_index = -1
        self.current_tile_type = -1
        self.current_detail_step = self.TILE_STEP_IDLE
        self.current_detail_progress = 0.0
        self.current_state = ""

        self.get_logger().info("\033[94m [1/4] [TASK MANAGER] initialize Done!\033[0m")
        self.get_logger().info(f"pick action    : {robot_ns}/tile/pick")
        self.get_logger().info(f"place action   : {robot_ns}/tile/place_press")
        self.get_logger().info(f"inspect service: {robot_ns}/tile/inspect")
        self.get_logger().info("kill service   : /kill")

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
            self.get_logger().warn("[KILL] cancel active sub goal requested")
            fut = self.active_sub_goal.cancel_goal_async()
            fut.add_done_callback(self._on_cancel_subgoal_done)
        except Exception as e:
            self.get_logger().error(f"[KILL] sub goal cancel failed: {e}")

    def _on_cancel_subgoal_done(self, future):
        try:
            _ = future.result()
            self.get_logger().warn("[KILL] sub goal cancel response received")
        except Exception as e:
            self.get_logger().error(f"[KILL] sub goal cancel response error: {e}")

    # --------------------------------------------------
    # /kill service
    # --------------------------------------------------
    def kill_service_callback(self, request, response):
        del request

        self.get_logger().warn("====================================")
        self.get_logger().warn("[KILL] /kill service called")
        self.get_logger().warn("====================================")

        self.kill_requested = True

        self._cancel_active_subgoal()
        self._publish_stop()

        response.success = True
        response.message = "kill requested: active subgoal cancel + /robot/command stop published"
        return response

    # --------------------------------------------------
    # goal / cancel
    # --------------------------------------------------
    def goal_callback(self, goal_request):
        n = len(goal_request.design_layout)

        if n <= 0:
            self.get_logger().warn("Reject: design_layout empty")
            return GoalResponse.REJECT

        if goal_request.completed_jobs < 0:
            self.get_logger().warn("Reject: completed_jobs < 0")
            return GoalResponse.REJECT

        if goal_request.current_tile_index < 0:
            self.get_logger().warn("Reject: current_tile_index < 0")
            return GoalResponse.REJECT

        if goal_request.current_tile_index > n:
            self.get_logger().warn("Reject: current_tile_index > len(design_layout)")
            return GoalResponse.REJECT

        if goal_request.completed_jobs > n:
            self.get_logger().warn("Reject: completed_jobs > len(design_layout)")
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
        self.get_logger().warn("Cancel request received on /task/run_job")
        self.kill_requested = True
        self._cancel_active_subgoal()
        self._publish_stop()
        return CancelResponse.ACCEPT

    # --------------------------------------------------
    # feedback helpers
    # --------------------------------------------------
    def _publish_execute_feedback(
        self,
        goal_handle,
        overall_step: int,
        overall_progress: float,
        tile_index: int,
        tile_type: int,
        detail_step: int,
        detail_progress: float,
        state: str,
    ):
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

        # tile당 PICK + PLACE + INSPECT = 3 units
        total_units = total_tiles * 3.0
        done_units = float(tile_index) * 3.0

        if tile_step == self.TILE_STEP_PICK:
            done_units += 0.0 + float(detail_progress) * 1.0
        elif tile_step == self.TILE_STEP_PLACE:
            done_units += 1.0 + float(detail_progress) * 1.0
        elif tile_step == self.TILE_STEP_INSPECT:
            done_units += 2.0 + float(detail_progress) * 1.0
        elif tile_step >= self.TILE_STEP_DONE:
            done_units += 3.0

        p = done_units / total_units
        return max(0.0, min(1.0, p))

    def _pick_feedback_cb(self, goal_handle, tile_index: int, tile_type: int, total_tiles: int):
        def _cb(feedback_msg):
            fb = feedback_msg.feedback
            sub_progress = float(getattr(fb, "progress", 0.0))
            sub_state = str(getattr(fb, "state", "pick"))

            overall_progress = self._calc_overall_progress(
                tile_index=tile_index,
                total_tiles=total_tiles,
                tile_step=self.TILE_STEP_PICK,
                detail_progress=sub_progress,
            )

            self.current_tile_index = tile_index
            self.current_tile_type = tile_type
            self.current_detail_step = self.TILE_STEP_PICK
            self.current_detail_progress = sub_progress
            self.current_state = f"pick:{sub_state}"

            self._publish_execute_feedback(
                goal_handle=goal_handle,
                overall_step=self.OVERALL_TILE_WORK,
                overall_progress=overall_progress,
                tile_index=tile_index,
                tile_type=tile_type,
                detail_step=self.TILE_STEP_PICK,
                detail_progress=sub_progress,
                state=f"pick:{sub_state}",
            )
        return _cb

    def _place_feedback_cb(self, goal_handle, tile_index: int, tile_type: int, total_tiles: int):
        def _cb(feedback_msg):
            fb = feedback_msg.feedback
            sub_progress = float(getattr(fb, "progress", 0.0))
            sub_state = str(getattr(fb, "state", "place"))

            overall_progress = self._calc_overall_progress(
                tile_index=tile_index,
                total_tiles=total_tiles,
                tile_step=self.TILE_STEP_PLACE,
                detail_progress=sub_progress,
            )

            self.current_tile_index = tile_index
            self.current_tile_type = tile_type
            self.current_detail_step = self.TILE_STEP_PLACE
            self.current_detail_progress = sub_progress
            self.current_state = f"place:{sub_state}"

            self._publish_execute_feedback(
                goal_handle=goal_handle,
                overall_step=self.OVERALL_TILE_WORK,
                overall_progress=overall_progress,
                tile_index=tile_index,
                tile_type=tile_type,
                detail_step=self.TILE_STEP_PLACE,
                detail_progress=sub_progress,
                state=f"place:{sub_state}",
            )
        return _cb

    def _publish_inspect_feedback(
        self,
        goal_handle,
        tile_index: int,
        tile_type: int,
        total_tiles: int,
        detail_progress: float,
        state: str,
    ):
        overall_progress = self._calc_overall_progress(
            tile_index=tile_index,
            total_tiles=total_tiles,
            tile_step=self.TILE_STEP_INSPECT,
            detail_progress=detail_progress,
        )

        self.current_tile_index = tile_index
        self.current_tile_type = tile_type
        self.current_detail_step = self.TILE_STEP_INSPECT
        self.current_detail_progress = detail_progress
        self.current_state = f"inspect:{state}"

        self._publish_execute_feedback(
            goal_handle=goal_handle,
            overall_step=self.OVERALL_TILE_WORK,
            overall_progress=overall_progress,
            tile_index=tile_index,
            tile_type=tile_type,
            detail_step=self.TILE_STEP_INSPECT,
            detail_progress=detail_progress,
            state=f"inspect:{state}",
        )

    # --------------------------------------------------
    # main execute
    # --------------------------------------------------
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

            if req.is_resume:
                start_tile_index = int(req.current_tile_index)
                start_step = int(req.current_step)
            else:
                start_tile_index = 0
                start_step = self.TILE_STEP_IDLE

            self.get_logger().info(
                f"[TASK] start token={req.token}, total_tiles={total_tiles}, "
                f"start_tile_index={start_tile_index}, start_step={start_step}"
            )

            if start_tile_index >= total_tiles:
                goal_handle.succeed()
                result.success = True
                result.message = "already_finished"
                return result

            for tile_index in range(start_tile_index, total_tiles):
                if self.kill_requested:
                    self.get_logger().warn("[TASK] killed before tile start")
                    goal_handle.abort()
                    result.success = False
                    result.message = "killed"
                    return result

                if goal_handle.is_cancel_requested:
                    self.get_logger().warn("[TASK] canceled before tile start")
                    goal_handle.canceled()
                    result.success = False
                    result.message = "canceled"
                    return result

                tile_type = int(layout[tile_index])

                if req.is_resume and tile_index == start_tile_index:
                    tile_step_start = int(start_step)
                else:
                    tile_step_start = self.TILE_STEP_IDLE

                self.get_logger().info(
                    f"[TASK] tile_index={tile_index}, "
                    f"tile_type={tile_type}, tile_step_start={tile_step_start}"
                )

                ok, msg = await self.run_single_tile(
                    goal_handle=goal_handle,
                    tile_index=tile_index,
                    tile_type=tile_type,
                    total_tiles=total_tiles,
                    tile_step_start=tile_step_start,
                )

                if not ok:
                    if self.kill_requested:
                        self.get_logger().warn(f"[TASK] aborted by kill: {msg}")
                        goal_handle.abort()
                        result.success = False
                        result.message = f"killed:{msg}"
                    elif goal_handle.is_cancel_requested:
                        self.get_logger().warn(f"[TASK] canceled: {msg}")
                        goal_handle.canceled()
                        result.success = False
                        result.message = "canceled"
                    else:
                        self.get_logger().error(f"[TASK] failed: {msg}")
                        goal_handle.abort()
                        result.success = False
                        result.message = msg
                    return result

            self._publish_execute_feedback(
                goal_handle=goal_handle,
                overall_step=self.OVERALL_FINISHED,
                overall_progress=1.0,
                tile_index=total_tiles - 1,
                tile_type=int(layout[-1]) if total_tiles > 0 else 0,
                detail_step=self.TILE_STEP_DONE,
                detail_progress=1.0,
                state="done",
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

    # --------------------------------------------------
    # run one tile
    # --------------------------------------------------
    async def run_single_tile(
        self,
        goal_handle,
        tile_index,
        tile_type,
        total_tiles,
        tile_step_start,
    ):
        # --------------------------
        # 1 PICK
        # --------------------------
        if tile_step_start <= self.TILE_STEP_PICK:
            ok, msg = await self.call_pick_tile(
                goal_handle=goal_handle,
                tile_index=tile_index,
                tile_type=tile_type,
            )
            if not ok:
                return False, f"pick_failed:{msg}"

        if self.kill_requested:
            return False, "killed_after_pick"

        if goal_handle.is_cancel_requested:
            return False, "canceled_after_pick"

        # --------------------------
        # 2 PLACE
        # --------------------------
        if tile_step_start <= self.TILE_STEP_PLACE:
            ok, msg = await self.call_place_tile(
                goal_handle=goal_handle,
                tile_index=tile_index,
                tile_type=tile_type,
            )
            if not ok:
                return False, f"place_failed:{msg}"

        if self.kill_requested:
            return False, "killed_after_place"

        if goal_handle.is_cancel_requested:
            return False, "canceled_after_place"

        # --------------------------
        # 3 INSPECT
        # --------------------------
        if tile_step_start <= self.TILE_STEP_INSPECT:
            ok, msg = await self.call_inspect(
                goal_handle=goal_handle,
                tile_index=tile_index,
                tile_type=tile_type,
                total_tiles=total_tiles,
            )
            if not ok:
                return False, f"inspect_failed:{msg}"

        if self.kill_requested:
            return False, "killed_after_inspect"

        if goal_handle.is_cancel_requested:
            return False, "canceled_after_inspect"

        self._publish_execute_feedback(
            goal_handle=goal_handle,
            overall_step=self.OVERALL_TILE_WORK,
            overall_progress=self._calc_overall_progress(
                tile_index=tile_index,
                total_tiles=total_tiles,
                tile_step=self.TILE_STEP_DONE,
                detail_progress=1.0,
            ),
            tile_index=tile_index,
            tile_type=tile_type,
            detail_step=self.TILE_STEP_DONE,
            detail_progress=1.0,
            state="tile_done",
        )

        return True, "ok"

    # --------------------------------------------------
    # pick action
    # --------------------------------------------------
    async def call_pick_tile(self, goal_handle, tile_index, tile_type):
        if self.kill_requested:
            return False, "killed_before_pick"

        if not self.pick_client.wait_for_server(timeout_sec=2.0):
            return False, "pick_action_server_unavailable"

        goal = PickTile.Goal()
        goal.tile_index = tile_index + 1
        goal.tile_type = tile_type

        send_future = self.pick_client.send_goal_async(
            goal,
            feedback_callback=self._pick_feedback_cb(
                goal_handle=goal_handle,
                tile_index=tile_index,
                tile_type=tile_type,
                total_tiles=self._current_total_tiles,
            ),
        )

        sub_goal_handle = await send_future
        self.active_sub_goal = sub_goal_handle

        if not sub_goal_handle.accepted:
            self.active_sub_goal = None
            return False, "pick_goal_rejected"

        if self.kill_requested:
            try:
                await sub_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self.active_sub_goal = None
            return False, "killed_during_pick_start"

        if goal_handle.is_cancel_requested:
            try:
                await sub_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self.active_sub_goal = None
            return False, "canceled"

        result_future = sub_goal_handle.get_result_async()
        result_wrap = await result_future
        self.active_sub_goal = None

        if result_wrap.status == GoalStatus.STATUS_CANCELED:
            if self.kill_requested:
                return False, "pick_canceled_by_kill"
            return False, "pick_canceled"

        result = result_wrap.result
        if not result.success:
            return False, result.message

        return True, result.message

    # --------------------------------------------------
    # place action
    # --------------------------------------------------
    async def call_place_tile(self, goal_handle, tile_index, tile_type):
        if self.kill_requested:
            return False, "killed_before_place"

        if not self.place_client.wait_for_server(timeout_sec=2.0):
            return False, "place_action_server_unavailable"

        goal = PlaceTile.Goal()
        goal.placement_index = tile_index + 1
        goal.max_press_force = 30.0
        goal.target_press_depth = 5.0

        send_future = self.place_client.send_goal_async(
            goal,
            feedback_callback=self._place_feedback_cb(
                goal_handle=goal_handle,
                tile_index=tile_index,
                tile_type=tile_type,
                total_tiles=self._current_total_tiles,
            ),
        )

        sub_goal_handle = await send_future
        self.active_sub_goal = sub_goal_handle

        if not sub_goal_handle.accepted:
            self.active_sub_goal = None
            return False, "place_goal_rejected"

        if self.kill_requested:
            try:
                await sub_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self.active_sub_goal = None
            return False, "killed_during_place_start"

        if goal_handle.is_cancel_requested:
            try:
                await sub_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self.active_sub_goal = None
            return False, "canceled"

        result_future = sub_goal_handle.get_result_async()
        result_wrap = await result_future
        self.active_sub_goal = None

        if result_wrap.status == GoalStatus.STATUS_CANCELED:
            if self.kill_requested:
                return False, "place_canceled_by_kill"
            return False, "place_canceled"

        result = result_wrap.result
        if not result.success:
            return False, result.message

        return True, result.message

    # --------------------------------------------------
    # inspect service
    # --------------------------------------------------
    async def call_inspect(self, goal_handle, tile_index, tile_type, total_tiles):
        del tile_type

        if self.kill_requested:
            return False, "killed_before_inspect"

        self._publish_inspect_feedback(
            goal_handle=goal_handle,
            tile_index=tile_index,
            tile_type=tile_type,
            total_tiles=total_tiles,
            detail_progress=0.0,
            state="waiting_inspect_service",
        )

        if not self.inspect_client.wait_for_service(timeout_sec=2.0):
            return False, "inspect_service_unavailable"

        if self.kill_requested:
            return False, "killed_before_inspect_call"

        if goal_handle.is_cancel_requested:
            return False, "canceled_before_inspect_call"

        req = Inspect.Request()

        self._publish_inspect_feedback(
            goal_handle=goal_handle,
            tile_index=tile_index,
            tile_type=tile_type,
            total_tiles=total_tiles,
            detail_progress=0.3,
            state="calling_inspect_service",
        )

        future = self.inspect_client.call_async(req)
        resp = await future

        if resp is None:
            return False, "inspect_response_none"

        if self.kill_requested:
            return False, "killed_after_inspect_call"

        if goal_handle.is_cancel_requested:
            return False, "canceled_after_inspect_call"

        if not resp.success:
            return False, resp.message

        self.get_logger().info(
            f"[TASK][INSPECT] tile_index={tile_index}, anomaly_scores={list(resp.anomaly_scores)}"
        )

        self._publish_inspect_feedback(
            goal_handle=goal_handle,
            tile_index=tile_index,
            tile_type=tile_type,
            total_tiles=total_tiles,
            detail_progress=1.0,
            state="inspect_done",
        )

        return True, resp.message


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