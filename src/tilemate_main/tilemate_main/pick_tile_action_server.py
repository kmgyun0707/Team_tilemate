#!/usr/bin/env python3
import time
import rclpy
import DR_init

from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor

from tilemate_msgs.action import PickTile
from tilemate_main.robot_config import RobotConfig, GripperConfig


class PickTileActionServer(Node):
    def __init__(self, robot_cfg: RobotConfig, gripper_cfg: GripperConfig, boot_node: Node):

        super().__init__("pick_tile_action_server", namespace=robot_cfg.robot_id)

        self.robot_cfg = robot_cfg
        self.gripper_cfg = gripper_cfg
        self._boot_node = boot_node

        self.cb_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            PickTile,
            "tile/pick",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group,
        )

        # OnRobot RG2
        from tilemate_main.onrobot import RG

        self.gripper = RG(
            gripper_cfg.GRIPPER_NAME,
            gripper_cfg.TOOLCHARGER_IP,
            gripper_cfg.TOOLCHARGER_PORT,
        )

        self.initialize_robot()
        self.get_logger().info("PickTileActionServer ready.")

    # --------------------------------------------------
    # Action callbacks
    # --------------------------------------------------

    def goal_callback(self, goal_request):

        if goal_request.tile_index < 0:
            self.get_logger().warn("Reject goal: tile_index < 0")
            return GoalResponse.REJECT

        if goal_request.tile_type <= 0:
            self.get_logger().warn("Reject goal: tile_type <= 0")
            return GoalResponse.REJECT

        self.get_logger().info(
            f"Goal accepted: tile_index={goal_request.tile_index}, "
            f"tile_type={goal_request.tile_type}"
        )

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):

        self.get_logger().warn("Cancel request received")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):

        result = PickTile.Result()

        try:
            ok, message = self.perform_task_once(goal_handle)

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = "canceled"
                return result

            if ok:
                goal_handle.succeed()
            else:
                goal_handle.abort()

            result.success = bool(ok)
            result.message = str(message)

            return result

        except Exception as e:

            self.get_logger().error(f"Execution failed: {e}")
            result.success = False
            result.message = f"exception:{e}"

            return result

    # --------------------------------------------------
    # Robot init
    # --------------------------------------------------

    def initialize_robot(self):

        from DSR_ROBOT2 import (
            set_tool,
            set_tcp,
            get_tool,
            get_tcp,
            ROBOT_MODE_MANUAL,
            ROBOT_MODE_AUTONOMOUS,
            get_robot_mode,
            set_robot_mode,
        )

        self.get_logger().info("[PICK_TILE] initialize_robot()")

        set_robot_mode(ROBOT_MODE_MANUAL)

        set_tool(self.robot_cfg.tool)
        set_tcp(self.robot_cfg.tcp)

        set_robot_mode(ROBOT_MODE_AUTONOMOUS)

        time.sleep(0.2)

        self.get_logger().info("#" * 50)
        self.get_logger().info(f"ROBOT_ID: {self.robot_cfg.robot_id}")
        self.get_logger().info(f"ROBOT_MODEL: {self.robot_cfg.robot_model}")
        self.get_logger().info(f"ROBOT_TCP: {get_tcp()}")
        self.get_logger().info(f"ROBOT_TOOL: {get_tool()}")
        self.get_logger().info(f"ROBOT_MODE: {get_robot_mode()}")
        self.get_logger().info(f"VELOCITY: {self.robot_cfg.vel}")
        self.get_logger().info(f"ACC: {self.robot_cfg.acc}")
        self.get_logger().info("#" * 50)

    # --------------------------------------------------
    # pick position preset
    # --------------------------------------------------

    def get_pick_pos(self, tile_type: int):

        from DSR_ROBOT2 import posx
        
        pick_positions = {
            1: [406.603, -401.761, 190.755, 37.71, -178.489, 126.707],  # 흰색 tile tray
            2: [314.41, -401.76, 190.755, 37.71, -178.489, 126.707],  # 검은색 tile tray
        }

        if tile_type not in pick_positions:
            raise ValueError(f"Unknown tile_type: {tile_type}")

        return posx(pick_positions[tile_type])

    # --------------------------------------------------
    # main pick logic
    # --------------------------------------------------

    def perform_task_once(self, goal_handle):

        from DSR_ROBOT2 import (
            posj,
            movej,
            movel,
            mwait,
            get_current_posx,
            posx,
            DR_BASE,
        )

        if goal_handle.is_cancel_requested:
            return False, "canceled"

        tile_type = int(goal_handle.request.tile_type)
        tile_index = int(goal_handle.request.tile_index)

        self.get_logger().info(
            f"[PICK_TILE] tile_index={tile_index}, tile_type={tile_type}"
        )

        # Home
        j_ready = posj([0, 0, 90, 0, 90, 0])
        movej(j_ready, vel=self.robot_cfg.vel, acc=self.robot_cfg.acc)
        self.gripper.open_gripper()
        mwait()

        # Pre pick
        pick_pos = self.get_pick_pos(tile_type)

        movel(pick_pos, vel=self.robot_cfg.vel, acc=self.robot_cfg.acc)
        mwait()

        # descend
        cur, _ = get_current_posx(DR_BASE)

        target = [
            cur[0],
            cur[1],
            cur[2] - 50,
            cur[3],
            cur[4],
            cur[5],
        ]

        movel(posx(target), ref=DR_BASE, vel=30, acc=30)
        mwait()

        # gripper close
        self.gripper.close_gripper()

        time.sleep(0.5)

        # lift
        movel(pick_pos, vel=30, acc=30)
        mwait()

        #move home
        movej(j_ready, vel=self.robot_cfg.vel, acc=self.robot_cfg.acc)


        return True, "pick_success"


# --------------------------------------------------
# main
# --------------------------------------------------

def main(args=None):

    rclpy.init(args=args)

    robot_cfg = RobotConfig()
    gripper_cfg = GripperConfig()

    boot = rclpy.create_node("dsr_boot_pick_tile", namespace=robot_cfg.robot_id)

    DR_init.__dsr__id = robot_cfg.robot_id
    DR_init.__dsr__model = robot_cfg.robot_model
    DR_init.__dsr__node = boot

    import DSR_ROBOT2

    node = PickTileActionServer(robot_cfg, gripper_cfg, boot)

    ex = SingleThreadedExecutor()
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