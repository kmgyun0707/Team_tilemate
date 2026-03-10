#!/usr/bin/env python3
import time
import rclpy
import DR_init

from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Bool
from tilemate_msgs.action import Cowork
from tilemate_main.robot_config import RobotConfig, GripperConfig


class CoworkActionServer(Node):
    def __init__(self, robot_cfg: RobotConfig, gripper_cfg: GripperConfig, boot_node: Node):
        super().__init__("cowork_action_server", namespace=robot_cfg.robot_id)

        self.robot_cfg = robot_cfg
        self.gripper_cfg = gripper_cfg
        self._boot_node = boot_node
        self.cb_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            Cowork,
            "tile/cowork",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group,
        )

        from tilemate_main.onrobot import RG
        self.gripper = RG(
            gripper_cfg.GRIPPER_NAME,
            gripper_cfg.TOOLCHARGER_IP,
            gripper_cfg.TOOLCHARGER_PORT,
        )

        self.human_take_confirmed = False
        self.cement_done_confirmed = False
        self.human_return_confirmed = False

        self.create_subscription(
            Bool,
            "cowork/human_take_confirm",
            self.human_take_callback,
            10,
            callback_group=self.cb_group,
        )

        self.create_subscription(
            Bool,
            "cowork/cement_done",
            self.cement_done_callback,
            10,
            callback_group=self.cb_group,
        )

        self.create_subscription(
            Bool,
            "cowork/human_return_confirm",
            self.human_return_callback,
            10,
            callback_group=self.cb_group,
        )

        self.initialize_robot()
        self.get_logger().info("\033[94m [3/5] [COWORK_TILE] initialize Done!\033[0m")

    # --------------------------------------------------
    # event callbacks
    # --------------------------------------------------

    def human_take_callback(self, msg: Bool):
        self.human_take_confirmed = bool(msg.data)
        self.get_logger().info(f"[COWORK] human_take_confirmed={self.human_take_confirmed}")

    def cement_done_callback(self, msg: Bool):
        self.cement_done_confirmed = bool(msg.data)
        self.get_logger().info(f"[COWORK] cement_done_confirmed={self.cement_done_confirmed}")

    def human_return_callback(self, msg: Bool):
        self.human_return_confirmed = bool(msg.data)
        self.get_logger().info(f"[COWORK] human_return_confirmed={self.human_return_confirmed}")

    # --------------------------------------------------
    # action callbacks
    # --------------------------------------------------

    def goal_callback(self, goal_request):
        if goal_request.tile_index < 0:
            self.get_logger().warn("[COWORK] Reject goal: tile_index < 0")
            return GoalResponse.REJECT

        if goal_request.tile_type < 0:
            self.get_logger().warn("[COWORK] Reject goal: tile_type < 0")
            return GoalResponse.REJECT

        self.get_logger().info(
            f"[COWORK] Goal accepted: tile_index={goal_request.tile_index}, "
            f"tile_type={goal_request.tile_type}, "
            f"job_token={goal_request.job_token}, "
            f"wait_cement_done={goal_request.wait_cement_done}"
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().warn("[COWORK] Cancel request received")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        result = Cowork.Result()
        self.reset_flags()

        try:
            ok, message, finished_step = self.perform_cowork_once(goal_handle)

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = "canceled"
                result.finished_step = finished_step
                result.tile_in_gripper = self.is_tile_in_gripper()
                return result

            if ok:
                goal_handle.succeed()
            else:
                goal_handle.abort()

            result.success = bool(ok)
            result.message = str(message)
            result.finished_step = int(finished_step)
            result.tile_in_gripper = self.is_tile_in_gripper()
            return result

        except Exception as e:
            self.get_logger().error(f"[COWORK] Execution failed: {e}")
            goal_handle.abort()
            result.success = False
            result.message = f"exception: {e}"
            result.finished_step = -1
            result.tile_in_gripper = self.is_tile_in_gripper()
            return result

    # --------------------------------------------------
    # helpers
    # --------------------------------------------------

    def reset_flags(self):
        self.human_take_confirmed = False
        self.cement_done_confirmed = False
        self.human_return_confirmed = False

    def publish_feedback(self, goal_handle, stage, detail, current_step):
        fb = Cowork.Feedback()
        fb.stage = str(stage)
        fb.detail = str(detail)
        fb.current_step = int(current_step)
        fb.human_take_confirmed = bool(self.human_take_confirmed)
        fb.cement_done_confirmed = bool(self.cement_done_confirmed)
        fb.human_return_confirmed = bool(self.human_return_confirmed)
        fb.tile_in_gripper = self.is_tile_in_gripper()
        goal_handle.publish_feedback(fb)

    def is_tile_in_gripper(self):
        # TODO: RG2 width/force 기반으로 실제 판정으로 교체
        return True

    def check_cancel(self, goal_handle, step_no):
        if goal_handle.is_cancel_requested:
            self.get_logger().warn(f"[COWORK] canceled at step={step_no}")
            return True
        return False

    def wait_for_flag(self, goal_handle, attr_name, timeout_sec, step_no, stage, detail):
        start = time.time()

        while time.time() - start < timeout_sec:
            if self.check_cancel(goal_handle, step_no):
                return False, "canceled"

            if getattr(self, attr_name):
                return True, "ok"

            self.publish_feedback(goal_handle, stage, detail, step_no)
            time.sleep(0.1)

        return False, f"timeout:{attr_name}"

    # --------------------------------------------------
    # robot init
    # --------------------------------------------------

    def initialize_robot(self):
        from DSR_ROBOT2 import (
            set_tool,
            set_tcp,
            ROBOT_MODE_MANUAL,
            ROBOT_MODE_AUTONOMOUS,
            set_robot_mode,
        )

        self.get_logger().info("[COWORK] initialize_robot()")

        set_robot_mode(ROBOT_MODE_MANUAL)
        set_tool(self.robot_cfg.tool)
        set_tcp(self.robot_cfg.tcp)
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        time.sleep(0.2)

    # --------------------------------------------------
    # motion helpers
    # --------------------------------------------------

    def move_relative(self, dx: float, dy: float, dz: float, dw: float = 0.0, dp: float = 0.0, dr: float = 0.0):
        from DSR_ROBOT2 import posx, movel, mwait, DR_BASE, get_current_posx

        cur, _ = get_current_posx(DR_BASE)
        target = [
            cur[0] + dx,
            cur[1] + dy,
            cur[2] + dz,
            cur[3] + dw,
            cur[4] + dp,
            cur[5] + dr,
        ]
        movel(posx(target), ref=DR_BASE, vel=30, acc=30)
        mwait()

    def move_absolute_keep_rpy_offset(self, x: float, y: float, z: float, w: float = 0.0, p: float = 0.0, r: float = 0.0):
        from DSR_ROBOT2 import posx, movel, mwait, DR_BASE, get_current_posx

        cur, _ = get_current_posx(DR_BASE)
        target = [
            x,
            y,
            z,
            cur[3] + w,
            cur[4] + p,
            cur[5] + r,
        ]
        movel(posx(target), ref=DR_BASE, vel=30, acc=30)
        mwait()

    def move_absolute_xyz_keep_rpy(self, xyz):
        from DSR_ROBOT2 import posx, movel, mwait, DR_BASE, get_current_posx

        cur, _ = get_current_posx(DR_BASE)
        target = [
            float(xyz[0]),
            float(xyz[1]),
            float(xyz[2]),
            cur[3],
            cur[4],
            cur[5],
        ]
        movel(posx(target), ref=DR_BASE, vel=30, acc=30)
        mwait()

    # --------------------------------------------------
    # tool pick sequence
    # --------------------------------------------------

    def pick_tool_sequence(self, goal_handle, start_step: int):
        from DSR_ROBOT2 import posx, movel, mwait, wait, DR_BASE, posj, movej

        # 기존 단독 스크립트 값 이식
        tool_pre_grip = [238.763, -373.264, 201.064]
        tool_grip = [235.122, -362.591, 123.0]
        retreat_x = [235.122, -232.591, 123.0]

        # step start_step: 툴 상단 이동
        if self.check_cancel(goal_handle, start_step):
            return False, "canceled", start_step

        self.publish_feedback(goal_handle, "MOVE_TOOL_PRE_GRIP", "move to tool pre-grip", start_step)
        self.get_logger().info(f"[COWORK] step{start_step}: move to tool_pre_grip={tool_pre_grip}")
        
        self.move_relative(0.0, 30.0, 100.0)
        


        j_ready = posj([0, 0, 90, 0, 90, 0])
        
        self.get_logger().info(" move home")
        movej(j_ready, vel=self.robot_cfg.vel, acc=self.robot_cfg.acc)
        mwait()

        # step start_step+3: 툴 파지 위치 이동
        if self.check_cancel(goal_handle, start_step + 1):
            return False, "canceled", start_step + 1

        self.publish_feedback(goal_handle, "MOVE_TOOL_GRIP", "move to tool grip pose", start_step + 1)
        self.get_logger().info(f"[COWORK] step{start_step+1}: move to tool_grip={tool_grip}")
        self.move_absolute_xyz_keep_rpy(tool_grip)

        # step start_step+4: 툴 파지
        if self.check_cancel(goal_handle, start_step + 2):
            return False, "canceled", start_step + 2

        self.publish_feedback(goal_handle, "GRIP_TOOL", "close gripper for tool pick", start_step + 2)
        self.get_logger().info(f"[COWORK] step{start_step+2}: close gripper")
        self.gripper.close_gripper()
        wait(1.5)

        # step start_step+5: 후퇴
        if self.check_cancel(goal_handle, start_step + 3):
            return False, "canceled", start_step + 3

        self.publish_feedback(goal_handle, "RETREAT_WITH_TOOL", "retreat after tool grip", start_step + 5)
        self.get_logger().info(f"[COWORK] step{start_step+3}: retreat to {retreat_x}")
        self.move_absolute_xyz_keep_rpy([retreat_x[0], retreat_x[1], retreat_x[2] + 10.0])

        # step start_step+6:  회전
        if self.check_cancel(goal_handle, start_step + 4):
            return False, "canceled", start_step + 4

        self.publish_feedback(goal_handle, "ROTATE_AFTER_TOOL_PICK", "rotate -180 deg after retreat", start_step + 6)
        self.get_logger().info(f"[COWORK] step{start_step+4}: rotate -180 deg after retreat")

        return True, "tool_pick_success", start_step + 4

    # --------------------------------------------------
    # main cowork logic
    # --------------------------------------------------

    def perform_cowork_once(self, goal_handle):
        from DSR_ROBOT2 import posx, movel, mwait, DR_BASE

        tile_type = int(goal_handle.request.tile_type)
        tile_index = int(goal_handle.request.tile_index)
        wait_cement_done = bool(goal_handle.request.wait_cement_done)

        self.get_logger().info(
            f"[COWORK] start tile_index={tile_index}, tile_type={tile_type}, "
            f"wait_cement_done={wait_cement_done}"
        )

        # step 1) 사람에게 건네줄 위치 이동
        if self.check_cancel(goal_handle, 1):
            return False, "canceled", 1

        give_pose = [517.21, -599.12, 306.50, 88.73, -90.0, -90.0]

        self.publish_feedback(goal_handle, "MOVE_TO_GIVE", "move to give pose", 1)
        self.get_logger().info(f"[COWORK] step1: move to give_pose={give_pose}")
        movel(posx(give_pose), ref=DR_BASE, vel=self.robot_cfg.vel, acc=self.robot_cfg.acc)
        mwait()

        # step 2) 사람이 가져갈 때까지 대기
        if self.check_cancel(goal_handle, 2):
            return False, "canceled", 2

        self.publish_feedback(goal_handle, "WAIT_HUMAN_TAKE", "waiting human take confirm", 2)
        ok, msg = self.wait_for_flag(
            goal_handle=goal_handle,
            attr_name="human_take_confirmed",
            timeout_sec=60.0,
            step_no=2,
            stage="WAIT_HUMAN_TAKE",
            detail="waiting human take confirm",
        )
        if not ok:
            return False, msg, 2

        # step 3) 타일 release
        if self.check_cancel(goal_handle, 3):
            return False, "canceled", 3

        self.publish_feedback(goal_handle, "RELEASE_TILE", "open gripper for human take", 3)
        self.get_logger().info("[COWORK] step3: open gripper")
        self.gripper.open_gripper()
        time.sleep(1.0)

        # step 4) 시멘트 작업 완료까지 대기
        if wait_cement_done:
            if self.check_cancel(goal_handle, 4):
                return False, "canceled", 4

            self.publish_feedback(goal_handle, "WAIT_CEMENT_DONE", "waiting cement done", 4)
            ok, msg = self.wait_for_flag(
                goal_handle=goal_handle,
                attr_name="cement_done_confirmed",
                timeout_sec=300.0,
                step_no=4,
                stage="WAIT_CEMENT_DONE",
                detail="waiting cement done confirm",
            )
            if not ok:
                return False, msg, 4
        else:
            self.get_logger().info("[COWORK] step4 skipped: wait_cement_done=False")

        # step 5~11) 툴 상단 이동 ~ 툴 파지 이식
        ok, msg, finished_step = self.pick_tool_sequence(goal_handle, start_step=5)
        if not ok:
            return False, msg, finished_step

        self.publish_feedback(goal_handle, "DONE", "cowork done", finished_step + 1)
        self.get_logger().info("[COWORK] finished")
        return True, "cowork_success", finished_step + 1


# --------------------------------------------------
# main
# --------------------------------------------------

def main(args=None):
    rclpy.init(args=args)

    robot_cfg = RobotConfig()
    gripper_cfg = GripperConfig()

    boot = rclpy.create_node("dsr_boot_cowork", namespace=robot_cfg.robot_id)

    DR_init.__dsr__id = robot_cfg.robot_id
    DR_init.__dsr__model = robot_cfg.robot_model
    DR_init.__dsr__node = boot

    import DSR_ROBOT2  # noqa: F401

    node = CoworkActionServer(robot_cfg, gripper_cfg, boot)

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
            boot.destroy_node()
        except Exception:
            pass

        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()