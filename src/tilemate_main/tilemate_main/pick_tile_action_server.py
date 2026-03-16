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
        self.get_logger().info("\033[94m [2/5] [PICK_TILE] initialize Done!\033[0m")

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
    def publish_feedback(
        self,
        goal_handle,
        step: int,
        progress: float,
        state: str,
        target_xyz=None,
    ):
        fb = PickTile.Feedback()
        fb.step = int(step)
        fb.progress = float(progress)
        fb.state = str(state)

        if target_xyz is None:
            fb.target_x = 0.0
            fb.target_y = 0.0
            fb.target_z = 0.0
        else:
            fb.target_x = float(target_xyz[0])
            fb.target_y = float(target_xyz[1])
            fb.target_z = float(target_xyz[2])

        goal_handle.publish_feedback(fb)
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
        depth_positions = {
            1: [359.315, -419.616, 210.136, 140.074, 179.203, -127.918],  # 흰색타일 tray 측정위치
            2: [439.062, -393.011, 210.746, 139.438, 179.149, 51.464],  # 검은색타일 tray 측정위치
        }
        if tile_type not in depth_positions:
            raise ValueError(f"Unknown tile_type: {tile_type}")
        
        return depth_positions[tile_type]
    
    def get_pick_above_pos(self, tile_type: int):
        pick_positions = {
            1: [436.158, -403.759, 200.0 , 170.548, 178.998, -98.723], # 흰색타일 중심점 상단
            2: [351.784, -406.931, 200.327, 141.574, 178.994, -127.464], # 검은색타일 중심점 상단
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
            DR_TOOL,
        )

        if goal_handle.is_cancel_requested:
            return False, "canceled"

        tile_type = int(goal_handle.request.tile_type)
        tile_index = int(goal_handle.request.tile_index)

        self.get_logger().info(f"[PICK_TILE] tile_index={tile_index}, tile_type={tile_type}")

        # 1) Home
        j_ready = posj([0, 0, 90, 0, 90, 0])
        self.publish_feedback(goal_handle, 1, 0.05, "move_home")
        self.get_logger().info("[PICK_TILE] step1: move home")
        movej(j_ready, vel=self.robot_cfg.vel, acc=self.robot_cfg.acc)
        mwait()
        self.gripper.open_gripper()

        if goal_handle.is_cancel_requested:
            return False, "canceled"

        # 2) depth 측정 위치 이동
        depth_above = self.get_depth_above_pos(tile_type)
        
        self.publish_feedback(goal_handle, 2, 0.20, "move_depth_above", depth_above[:3])
        self.get_logger().info(f"[PICK_TILE] step2: move to depth_above={depth_above}")
        movel(posx(depth_above), ref=DR_BASE, vel=self.robot_cfg.vel, acc=self.robot_cfg.acc)
        mwait()

        if goal_handle.is_cancel_requested:
            return False, "canceled"

        # 3) depth 준비 대기
        self.publish_feedback(goal_handle, 3, 0.30, "wait_depth_ready", depth_above[:3])
        self.get_logger().info("[PICK_TILE] step3: wait for depth ready")
        if not self.wait_for_depth_ready(timeout_sec=2.0):
            self.publish_feedback(goal_handle, 3, 0.30, "depth_not_ready", depth_above[:3])
            return False, "depth_not_ready"

        # 4) 현재 TCP 기준 depth target 추정
        robot_posx, _ = get_current_posx(DR_BASE)
        self.publish_feedback(goal_handle, 4, 0.45, "estimate_target")
        self.get_logger().info(f"[PICK_TILE] step4: current tcp={robot_posx}")

        base_point = self.depth_localizer.estimate_center_pick_base_point(
            robot_posx=robot_posx,
            kernel_size=5,
            min_mm=200,
            max_mm=5000,
            inlier_thresh_mm=80,
            z_offset_mm=-43.0,
        )

        if base_point is None:
            self.publish_feedback(goal_handle, 4, 0.45, "depth_target_not_found")
            return False, "depth_target_not_found"

        # camera depth 원본값(mm)
        filtered_depth = getattr(self.depth_localizer, "last_filtered_depth", None)

        if filtered_depth is None:
            self.publish_feedback(goal_handle, 4, 0.45, "depth_value_missing")
            self.get_logger().warn("[PICK_TILE] filtered_depth is None")
            return False, "depth_value_missing"

        target_xyz = [
            float(base_point[0]),
            float(base_point[1]),
            float(base_point[2]),
        ]

        self.get_logger().info(
            f"[PICK_TILE] step4: estimated base_point={base_point.tolist()}, "
            f"filtered_depth={filtered_depth:.1f} mm"
        )

        # --------------------------------------------------
        # 타일 존재 여부 판단: camera filtered_depth 기준
        # --------------------------------------------------
        if float(filtered_depth) >= 401.0:
            self.get_logger().warn(
                f"[PICK_TILE] no more tiles!: filtered_depth={filtered_depth:.1f} >= 401.0"
            )

            self.publish_feedback(goal_handle, 4, 0.45, "tile_not_found", target_xyz)

            # 홈 복귀
            j_ready = posj([0, 0, 90, 0, 90, 0])
            movej(j_ready, vel=self.robot_cfg.vel, acc=self.robot_cfg.acc)
            mwait()

            return False, f"tile_not_found_depth_{filtered_depth:.1f}"

        # 5) pick 상단 이동
        pick_above = self.get_pick_above_pos(tile_type)
        self.publish_feedback(goal_handle, 5, 0.60, "move_pick_above", pick_above[:3])
        self.get_logger().info(f"[PICK_TILE] step5: move to pick_above={pick_above}")
        movel(posx(pick_above), ref=DR_BASE, vel=self.robot_cfg.vel, acc=self.robot_cfg.acc)
        mwait()

        if goal_handle.is_cancel_requested:
            return False, "canceled"

        # 6) 실제 pick 위치로 하강
        pick_coordinate = self.get_pick_above_pos(tile_type)
        pick_coordinate[2] = float(base_point[2])

        self.publish_feedback(goal_handle, 6, 0.75, "descend_pick_target", target_xyz)
        self.get_logger().info(f"[PICK_TILE] step6: descend to pick_target={pick_coordinate}")
        movel(posx(pick_coordinate), ref=DR_BASE, vel=20, acc=20)
        mwait()

        if goal_handle.is_cancel_requested:
            return False, "canceled"

        # 7) gripper close
        self.publish_feedback(goal_handle, 7, 0.88, "close_gripper", target_xyz)
        self.get_logger().info("[PICK_TILE] step7: close gripper")
        self.gripper.close_gripper()
        time.sleep(1.5)

        if goal_handle.is_cancel_requested:
            return False, "canceled"

        # 8) 다시 상단으로 상승
        self.publish_feedback(goal_handle, 8, 0.97, "lift_after_pick", pick_above[:3])
        self.get_logger().info(f"[PICK_TILE] step8: lift to approach={pick_above}")
        movel(posx(pick_above), ref=DR_BASE, vel=30, acc=30)
        mwait()

        self.publish_feedback(goal_handle, 9, 1.0, "done", pick_above[:3])
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