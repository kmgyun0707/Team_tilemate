#!/usr/bin/env python3
import os
import time
import rclpy
import DR_init

from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory

from tilemate_msgs.action import PickTile
from tilemate_main.robot_config import RobotConfig, GripperConfig
from tilemate_main.depth_localizer import DepthLocalizer


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

        from tilemate_main.onrobot import RG

        self.gripper = RG(
            gripper_cfg.GRIPPER_NAME,
            gripper_cfg.TOOLCHARGER_IP,
            gripper_cfg.TOOLCHARGER_PORT,
        )

        self.initialize_robot()

        package_path = get_package_share_directory("tilemate_main")
        gripper2cam_path = os.path.join(package_path, "resource", "T_gripper2camera.npy")

        self.depth_localizer = DepthLocalizer(
            node=self,
            gripper2cam_path=gripper2cam_path,
            use_inverse=False,
        )

        self.get_logger().info(f"[PICK_TILE] gripper2cam_path={gripper2cam_path}")
        self.get_logger().info("[PICK_TILE] initialize done")

    # --------------------------------------------------
    # Action callbacks
    # --------------------------------------------------

    def goal_callback(self, goal_request):
        if goal_request.tile_index < 0:
            self.get_logger().warn("Reject goal: tile_index < 0")
            return GoalResponse.REJECT

        if goal_request.tile_type < 0:
            self.get_logger().warn("Reject goal: tile_type < 0")
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
            ROBOT_MODE_MANUAL,
            ROBOT_MODE_AUTONOMOUS,
            set_robot_mode,
        )

        self.get_logger().info("[PICK_TILE] initialize_robot()")

        set_robot_mode(ROBOT_MODE_MANUAL)
        set_tool(self.robot_cfg.tool)
        set_tcp(self.robot_cfg.tcp)
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)

        time.sleep(0.2)

    # --------------------------------------------------
    # pick position preset
    # --------------------------------------------------

    def get_depth_above_pos(self, tile_type: int):
        from DSR_ROBOT2 import posx
        depth_positions = {
            0: [406.603, -327.859, 190.755, 63.012, 179.998, -116.015],  # 흰색타일 tray 측정위치
            1: [314.410, -327.859, 190.755, 63.012, 179.998, -116.015],  # 검은색타일 tray 측정위치
        }
        if tile_type not in depth_positions:
            raise ValueError(f"Unknown tile_type: {tile_type}")
        
        return depth_positions[tile_type]
    
    def get_pick_above_pos(self, tile_type: int):
        from DSR_ROBOT2 import posx
        pick_positions = {
            0: [409.182, -399.668, 190.755, 128.103, -178.027, 40.091], # 흰색타일 중심점 상단
            1: [325.568, -406.84, 190.755, 1.445, -179.276, -89.259], # 검은색타일 중심점 상단
        }

        if tile_type not in pick_positions:
            raise ValueError(f"Unknown tile_type: {tile_type}")

        return pick_positions[tile_type]

    def wait_for_depth_ready(self, timeout_sec=2.0):
        start = time.time()
        while time.time() - start < timeout_sec:
            if self.depth_localizer.is_ready():
                return True
            time.sleep(0.05)
        return False

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

        self.get_logger().info(f"[PICK_TILE] tile_index={tile_index}, tile_type={tile_type}")

        # 1) Home
        j_ready = posj([0, 0, 90, 0, 90, 0])
        self.get_logger().info("[PICK_TILE] step1: move home")
        movej(j_ready, vel=self.robot_cfg.vel, acc=self.robot_cfg.acc)
        mwait()
        self.gripper.open_gripper()

        if goal_handle.is_cancel_requested:
            return False, "canceled"

        # 2) 측정위치 이동
        depth_above = self.get_depth_above_pos(tile_type)
        self.get_logger().info(f"[PICK_TILE] step2: move to depth_above={depth_above}")
        movel(depth_above, ref=DR_BASE, vel=self.robot_cfg.vel, acc=self.robot_cfg.acc)
        mwait()

        if goal_handle.is_cancel_requested:
            return False, "canceled"

        # 3) depth 준비 확인
        self.get_logger().info("[PICK_TILE] step3: wait for depth ready")
        if not self.wait_for_depth_ready(timeout_sec=2.0):
            return False, "depth_not_ready"

        # 4) 현재 TCP pose 기준으로 center depth -> base point 추정
        robot_posx, _ = get_current_posx(DR_BASE)
        self.get_logger().info(f"[PICK_TILE] step4: current tcp={robot_posx}")

        base_point = self.depth_localizer.estimate_center_pick_base_point(
            robot_posx=robot_posx,
            kernel_size=5,
            min_mm=200,
            max_mm=5000,
            inlier_thresh_mm=80,
            z_offset_mm=+50.0, #TODO 그리퍼 하강 위치 조절 필요 
        )

        if base_point is None:
            return False, "depth_target_not_found"

        self.get_logger().info(f"[PICK_TILE] step4: estimated base_point={base_point.tolist()}")

        # orientation은 현재 pick_above 자세 유지

        target_coordinate = [
            float(base_point[0]),
            float(base_point[1]),
            float(base_point[2]),
            robot_posx[3],
            robot_posx[4],
            robot_posx[5]+90.0,
        ]

        # 2) tray 상단으로 이동
        pick_above = self.get_pick_above_pos(tile_type)

        self.get_logger().info(f"[PICK_TILE] step2: move to pick_above={pick_above}")
        movel(posx(pick_above), ref=DR_BASE, vel=self.robot_cfg.vel, acc=self.robot_cfg.acc)
        mwait()


        if goal_handle.is_cancel_requested:
            return False, "canceled"

        # 6) 실제 pick 위치로 하강
        pick_coordinate = self.get_pick_above_pos(tile_type)
        pick_coordinate[2] = float(base_point[2])
        self.get_logger().info(f"[PICK_TILE] step6: descend to pick_target={pick_coordinate}")
        movel(posx(pick_coordinate), ref=DR_BASE, vel=20, acc=20)
        mwait()

        if goal_handle.is_cancel_requested:
            return False, "canceled"

        # 7) gripper close
        self.get_logger().info("[PICK_TILE] step7: close gripper")
        self.gripper.close_gripper()
        time.sleep(1.5)

        # 8) 다시 상단으로 상승
        self.get_logger().info(f"[PICK_TILE] step8: lift to approach={pick_above}")
        movel(posx(pick_above), ref=DR_BASE, vel=30, acc=30)
        mwait()

        # TODO 툴에 배치

        # movel(posx(tool_coordinate), ref=DR_BASE, vel=30, acc=30)
        # TODO 툴 잡고 배치 위치 이동
        # 5번 조인트 movej()


        self.get_logger().info("[PICK_TILE] finished")
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

    import DSR_ROBOT2  # noqa: F401

    node = PickTileActionServer(robot_cfg, gripper_cfg, boot)

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