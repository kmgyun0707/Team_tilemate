#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from tilemate_msgs.action import TaskJob, PickTile, PlaceTile, InspectTile, CompactTile

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

        # 상위 작업 액션 서버
        self._action_server = ActionServer(
            self,
            TaskJob,
            "task/run_job",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group,
        )

        # 하위 액션 클라이언트
        self.pick_client = ActionClient(
            self, PickTile, "tile/pick", callback_group=self.cb_group
        )
        self.place_client = ActionClient(
            self, PlaceTile, "tile/place_press", callback_group=self.cb_group
        )
        self.inspect_client = ActionClient(
            self, InspectTile, "tile/inspect", callback_group=self.cb_group
        )
        self.compact_client = ActionClient(
            self, CompactTile, "tile/compact", callback_group=self.cb_group
        )

        self.get_logger().info("TaskManagerNode ready")

    # --------------------------------------------------
    # goal / cancel
    # --------------------------------------------------
    def goal_callback(self, goal_request):
        n = len(goal_request.design_layout)

        if n <= 0:
            self.get_logger().warn("Reject: design_layout is empty")
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
            f"current_tile_index={goal_request.current_tile_index}, "
            f"current_step={goal_request.current_step}"
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().warn("Cancel request received")
        return CancelResponse.ACCEPT

    # --------------------------------------------------
    # feedback helper
    # --------------------------------------------------
    def publish_feedback(
        self,
        goal_handle,
        overall_step: int,
        overall_progress: float,
        tile_index: int,
        tile_type: int,
        tile_step: int,
        tile_progress: float,
        state: str,
    ):
        fb = TaskJob.Feedback()
        fb.overall_step = int(overall_step)
        fb.overall_progress = float(overall_progress)
        fb.tile_index = int(tile_index)
        fb.tile_type = int(tile_type)
        fb.tile_step = int(tile_step)
        fb.tile_progress = float(tile_progress)
        fb.state = str(state)
        goal_handle.publish_feedback(fb)

    # --------------------------------------------------
    # main execute
    # --------------------------------------------------
    async def execute_callback(self, goal_handle):
        result = TaskJob.Result()
        req = goal_handle.request

        try:
            layout = list(req.design_layout)
            total_tiles = len(layout)

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

            # 전체 타일 루프
            for tile_index in range(start_tile_index, total_tiles):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = "canceled"
                    return result

                tile_type = int(layout[tile_index])

                # resume 시작 타일이면 current_step부터,
                # 이후 타일이면 처음부터
                if req.is_resume and tile_index == start_tile_index:
                    tile_step_start = start_step
                else:
                    tile_step_start = self.TILE_STEP_IDLE

                self.get_logger().info(
                    f"[TASK] tile_index={tile_index}, tile_type={tile_type}, "
                    f"tile_step_start={tile_step_start}"
                )

                ok, msg = await self.run_single_tile(
                    goal_handle=goal_handle,
                    token=req.token,
                    tile_index=tile_index,
                    tile_type=tile_type,
                    total_tiles=total_tiles,
                    tile_step_start=tile_step_start,
                )

                if not ok:
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        result.success = False
                        result.message = "canceled"
                    else:
                        goal_handle.abort()
                        result.success = False
                        result.message = msg
                    return result

            # 완료
            self.publish_feedback(
                goal_handle=goal_handle,
                overall_step=self.OVERALL_FINISHED,
                overall_progress=1.0,
                tile_index=total_tiles - 1,
                tile_type=int(layout[-1]),
                tile_step=self.TILE_STEP_DONE,
                tile_progress=1.0,
                state="finished",
            )

            goal_handle.succeed()
            result.success = True
            result.message = "all_tiles_completed"
            return result

        except Exception as e:
            self.get_logger().error(f"[TASK] execute failed: {e}")
            goal_handle.abort()
            result.success = False
            result.message = f"exception:{e}"
            return result

    # --------------------------------------------------
    # run one tile
    # --------------------------------------------------
    async def run_single_tile(
        self,
        goal_handle,
        token: str,
        tile_index: int,
        tile_type: int,
        total_tiles: int,
        tile_step_start: int,
    ):
        # 진행률 계산 helper
        def overall_progress_of(tile_prog: float) -> float:
            return min((tile_index + tile_prog) / max(total_tiles, 1), 1.0)

        # --------------------------
        # 1) PLACE
        # --------------------------
        if tile_step_start <= self.TILE_STEP_PLACE:
            self.publish_feedback(
                goal_handle=goal_handle,
                overall_step=self.OVERALL_TILE_WORK,
                overall_progress=overall_progress_of(0.10),
                tile_index=tile_index,
                tile_type=tile_type,
                tile_step=self.TILE_STEP_PLACE,
                tile_progress=0.10,
                state="place_start",
            )

            ok, msg = await self.call_place_tile(
                goal_handle=goal_handle,
                tile_index=tile_index,
                tile_type=tile_type,
            )
            if not ok:
                return False, f"place_failed:{msg}"

        if goal_handle.is_cancel_requested:
            return False, "canceled"

        # --------------------------
        # 2) INSPECT
        # --------------------------
        if tile_step_start <= self.TILE_STEP_INSPECT:
            self.publish_feedback(
                goal_handle=goal_handle,
                overall_step=self.OVERALL_TILE_WORK,
                overall_progress=overall_progress_of(0.55),
                tile_index=tile_index,
                tile_type=tile_type,
                tile_step=self.TILE_STEP_INSPECT,
                tile_progress=0.55,
                state="inspect_start",
            )

            ok, msg = await self.call_inspect_tile(
                goal_handle=goal_handle,
                tile_index=tile_index,
                tile_type=tile_type,
            )
            if not ok:
                return False, f"inspect_failed:{msg}"

        if goal_handle.is_cancel_requested:
            return False, "canceled"

        # --------------------------
        # 3) COMPACT
        # --------------------------
        if tile_step_start <= self.TILE_STEP_COMPACT:
            self.publish_feedback(
                goal_handle=goal_handle,
                overall_step=self.OVERALL_TILE_WORK,
                overall_progress=overall_progress_of(0.80),
                tile_index=tile_index,
                tile_type=tile_type,
                tile_step=self.TILE_STEP_COMPACT,
                tile_progress=0.80,
                state="compact_start",
            )

            ok, msg = await self.call_compact_tile(
                goal_handle=goal_handle,
                tile_index=tile_index,
                tile_type=tile_type,
            )
            if not ok:
                return False, f"compact_failed:{msg}"

        # tile done
        self.publish_feedback(
            goal_handle=goal_handle,
            overall_step=self.OVERALL_TILE_WORK,
            overall_progress=overall_progress_of(1.0),
            tile_index=tile_index,
            tile_type=tile_type,
            tile_step=self.TILE_STEP_DONE,
            tile_progress=1.0,
            state="tile_done",
        )
        return True, "ok"

    # --------------------------------------------------
    # sub action calls
    # --------------------------------------------------
    async def call_place_tile(self, goal_handle, tile_index: int, tile_type: int):
        if not self.place_client.wait_for_server(timeout_sec=2.0):
            return False, "place_action_server_unavailable"

        goal = PlaceTile.Goal()

        # 예시: tile_type에 따라 placement index나 힘 파라미터를 다르게 줄 수 있음
        goal.placement_index = tile_index + 1
        goal.max_press_force = 15.0 if tile_type == 1 else 20.0
        goal.target_press_depth = 5.0 if tile_type == 1 else 6.0

        send_future = self.place_client.send_goal_async(
            goal,
            feedback_callback=lambda fb: self._fb_place(
                parent_goal_handle=goal_handle,
                tile_index=tile_index,
                tile_type=tile_type,
                fb_msg=fb,
            ),
        )
        sub_goal_handle = await send_future

        if not sub_goal_handle.accepted:
            return False, "place_goal_rejected"

        if goal_handle.is_cancel_requested:
            try:
                await sub_goal_handle.cancel_goal_async()
            except Exception:
                pass
            return False, "canceled"

        result_future = sub_goal_handle.get_result_async()
        result_wrap = await result_future
        result = result_wrap.result

        if not result.success:
            return False, result.message

        return True, result.message

    async def call_inspect_tile(self, goal_handle, tile_index: int, tile_type: int):
        if not self.inspect_client.wait_for_server(timeout_sec=2.0):
            return False, "inspect_action_server_unavailable"

        goal = InspectTile.Goal()
        goal.tile_index = tile_index + 1
        goal.tile_type = tile_type

        send_future = self.inspect_client.send_goal_async(
            goal,
            feedback_callback=lambda fb: self._fb_inspect(
                parent_goal_handle=goal_handle,
                tile_index=tile_index,
                tile_type=tile_type,
                fb_msg=fb,
            ),
        )
        sub_goal_handle = await send_future

        if not sub_goal_handle.accepted:
            return False, "inspect_goal_rejected"

        if goal_handle.is_cancel_requested:
            try:
                await sub_goal_handle.cancel_goal_async()
            except Exception:
                pass
            return False, "canceled"

        result_future = sub_goal_handle.get_result_async()
        result_wrap = await result_future
        result = result_wrap.result

        if not result.success:
            return False, result.message

        return True, result.message

    async def call_compact_tile(self, goal_handle, tile_index: int, tile_type: int):
        if not self.compact_client.wait_for_server(timeout_sec=2.0):
            return False, "compact_action_server_unavailable"

        goal = CompactTile.Goal()
        goal.tile_index = tile_index + 1
        goal.tile_type = tile_type

        send_future = self.compact_client.send_goal_async(
            goal,
            feedback_callback=lambda fb: self._fb_compact(
                parent_goal_handle=goal_handle,
                tile_index=tile_index,
                tile_type=tile_type,
                fb_msg=fb,
            ),
        )
        sub_goal_handle = await send_future

        if not sub_goal_handle.accepted:
            return False, "compact_goal_rejected"

        if goal_handle.is_cancel_requested:
            try:
                await sub_goal_handle.cancel_goal_async()
            except Exception:
                pass
            return False, "canceled"

        result_future = sub_goal_handle.get_result_async()
        result_wrap = await result_future
        result = result_wrap.result

        if not result.success:
            return False, result.message

        return True, result.message

    # --------------------------------------------------
    # sub feedback mapping
    # --------------------------------------------------
    def _fb_place(self, parent_goal_handle, tile_index: int, tile_type: int, fb_msg):
        fb = fb_msg.feedback

        state = getattr(fb, "state", "placing")
        tile_progress = float(getattr(fb, "progress", 0.3))

        self.publish_feedback(
            goal_handle=parent_goal_handle,
            overall_step=self.OVERALL_TILE_WORK,
            overall_progress=0.0,   # 세부 계산은 execute에서 이미 대략 주므로 여기선 단순화
            tile_index=tile_index,
            tile_type=tile_type,
            tile_step=self.TILE_STEP_PLACE,
            tile_progress=tile_progress,
            state=f"place:{state}",
        )

    def _fb_inspect(self, parent_goal_handle, tile_index: int, tile_type: int, fb_msg):
        fb = fb_msg.feedback
        state = getattr(fb, "state", "inspecting")
        tile_progress = float(getattr(fb, "progress", 0.7))

        self.publish_feedback(
            goal_handle=parent_goal_handle,
            overall_step=self.OVERALL_TILE_WORK,
            overall_progress=0.0,
            tile_index=tile_index,
            tile_type=tile_type,
            tile_step=self.TILE_STEP_INSPECT,
            tile_progress=tile_progress,
            state=f"inspect:{state}",
        )

    def _fb_compact(self, parent_goal_handle, tile_index: int, tile_type: int, fb_msg):
        fb = fb_msg.feedback
        state = getattr(fb, "state", "compacting")
        tile_progress = float(getattr(fb, "progress", 0.9))

        self.publish_feedback(
            goal_handle=parent_goal_handle,
            overall_step=self.OVERALL_TILE_WORK,
            overall_progress=0.0,
            tile_index=tile_index,
            tile_type=tile_type,
            tile_step=self.TILE_STEP_COMPACT,
            tile_progress=tile_progress,
            state=f"compact:{state}",
        )


def main(args=None):
    rclpy.init(args=args)

    node = TaskManagerNode()
    ex = MultiThreadedExecutor()

    ex.add_node(node)

    try:
        ex.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
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