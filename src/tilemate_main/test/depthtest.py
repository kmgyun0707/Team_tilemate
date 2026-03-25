#!/usr/bin/env python3
import os
import sys
import numpy as np
import cv2
from scipy.spatial.transform import Rotation

from ament_index_python import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

import DR_init

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

rclpy.init()
dsr_node = rclpy.create_node("rokey_simple_move", namespace=ROBOT_ID)
DR_init.__dsr__node = dsr_node
package_path = get_package_share_directory("tilemate_main")

try:
    from DSR_ROBOT2 import get_current_posx
except ImportError as e:
    print(f"Error importing DSR_ROBOT2: {e}")
    sys.exit()


class CenterDepthToBaseNode(Node):
    def __init__(self):
        super().__init__("center_depth_to_base_node")

        self.depth_image = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.window_name = "Depth Heatmap + Base Coord"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 1280, 720)

        self.create_subscription(
            Image,
            "/camera/camera/depth/image_rect_raw",
            self.depth_callback,
            10
        )

        self.create_subscription(
            CameraInfo,
            "/camera/camera/depth/camera_info",
            self.camera_info_callback,
            10
        )

        self.timer = self.create_timer(0.2, self.process_center_point)

        # 실제 경로로 수정
        self.gripper2cam_path = os.path.join(package_path, "resource", "T_gripper2camera.npy")
        self.last_filtered_depth = None
        self.last_camera_point = None
        self.last_base_point_direct = None
        self.last_base_point_inverse = None
        self.last_robot_posx = None

        self.get_logger().info("CenterDepthToBaseNode started.")

    def camera_info_callback(self, msg: CameraInfo):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def depth_callback(self, msg: Image):
        if msg.encoding != "16UC1":
            self.get_logger().error(f"Unsupported depth encoding: {msg.encoding}")
            return

        try:
            self.depth_image = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        except Exception as e:
            self.get_logger().error(f"Depth reshape failed: {e}")
            self.depth_image = None

    def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
        R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T

    def pixel_to_camera_3d(self, u, v, depth_mm, fx, fy, cx, cy):
        x = (u - cx) * depth_mm / fx
        y = (v - cy) * depth_mm / fy
        z = depth_mm
        return np.array([x, y, z], dtype=np.float64)

    def transform_camera_to_base_direct(self, camera_coords, gripper2cam_path, robot_pos):
        """
        파일을 그대로 사용:
        P_base = T_base_gripper @ T_gripper_camera @ P_camera
        """
        gripper2cam = np.load(gripper2cam_path)
        coord = np.append(np.array(camera_coords, dtype=np.float64), 1.0)

        x, y, z, rx, ry, rz = robot_pos
        base2gripper = self.get_robot_pose_matrix(x, y, z, rx, ry, rz)

        base2cam = base2gripper @ gripper2cam
        base_coord = base2cam @ coord
        return base_coord[:3]

    def transform_camera_to_base_inverse(self, camera_coords, gripper2cam_path, robot_pos):
        """
        파일이 반대 방향일 가능성 대비:
        P_base = T_base_gripper @ inv(T_gripper_camera) @ P_camera
        """
        gripper2cam = np.load(gripper2cam_path)
        cam2gripper = np.linalg.inv(gripper2cam)
        coord = np.append(np.array(camera_coords, dtype=np.float64), 1.0)

        x, y, z, rx, ry, rz = robot_pos
        base2gripper = self.get_robot_pose_matrix(x, y, z, rx, ry, rz)

        base_coord = base2gripper @ cam2gripper @ coord
        return base_coord[:3]

    def get_center_filtered_depth(self):
        if self.depth_image is None:
            return None, None, None, None

        h, w = self.depth_image.shape
        u = w // 2
        v = h // 2

        patch = self.depth_image[max(0, v - 2):min(h, v + 3), max(0, u - 2):min(w, u + 3)]
        valid = patch[(patch > 200) & (patch < 5000)]

        if valid.size == 0:
            return None, u, v, patch

        median_depth = np.median(valid)
        inliers = valid[np.abs(valid - median_depth) <= 100]

        if inliers.size > 0:
            filtered_depth = float(np.median(inliers))
        else:
            filtered_depth = float(median_depth)

        return filtered_depth, u, v, patch

    def make_heatmap(self, depth_image):
        valid_mask = (depth_image > 200) & (depth_image < 5000)

        if np.any(valid_mask):
            valid_pixels = depth_image[valid_mask].astype(np.float32)
            vis_min = np.percentile(valid_pixels, 5)
            vis_max = np.percentile(valid_pixels, 95)

            if vis_max <= vis_min:
                vis_max = vis_min + 1.0

            vis = depth_image.astype(np.float32)
            vis = np.clip(vis, vis_min, vis_max)
            vis = ((vis - vis_min) / (vis_max - vis_min) * 255.0).astype(np.uint8)

            heatmap = cv2.applyColorMap(vis, cv2.COLORMAP_JET)
            heatmap[~valid_mask] = (0, 0, 0)
        else:
            heatmap = np.zeros((depth_image.shape[0], depth_image.shape[1], 3), dtype=np.uint8)

        return heatmap

    def draw_overlay(self, heatmap, u, v, filtered_depth, camera_point, base_point_direct, base_point_inverse, robot_posx):
        h, w, _ = heatmap.shape

        x1 = max(0, u - 2)
        y1 = max(0, v - 2)
        x2 = min(w - 1, u + 2)
        y2 = min(h - 1, v + 2)

        cv2.circle(heatmap, (u, v), 4, (255, 255, 255), -1)
        cv2.rectangle(heatmap, (x1, y1), (x2, y2), (255, 255, 255), 1)

        panel_h = 210
        overlay = heatmap.copy()
        cv2.rectangle(overlay, (10, 10), (760, 10 + panel_h), (0, 0, 0), -1)
        heatmap[:] = cv2.addWeighted(overlay, 0.45, heatmap, 0.55, 0)

        lines = []
        lines.append(f"Center pixel : ({u}, {v})")

        if filtered_depth is not None:
            lines.append(f"Depth       : {filtered_depth:.1f} mm  ({filtered_depth/1000.0:.3f} m)")
        else:
            lines.append("Depth       : invalid")

        if camera_point is not None:
            lines.append(
                f"Camera XYZ  : X={camera_point[0]:.1f}, Y={camera_point[1]:.1f}, Z={camera_point[2]:.1f} mm"
            )
        else:
            lines.append("Camera XYZ  : invalid")

        if base_point_direct is not None:
            lines.append(
                f"Base XYZ(D) : X={base_point_direct[0]:.1f}, Y={base_point_direct[1]:.1f}, Z={base_point_direct[2]:.1f} mm"
            )
        else:
            lines.append("Base XYZ(D) : invalid")

        if base_point_inverse is not None:
            lines.append(
                f"Base XYZ(I) : X={base_point_inverse[0]:.1f}, Y={base_point_inverse[1]:.1f}, Z={base_point_inverse[2]:.1f} mm"
            )
        else:
            lines.append("Base XYZ(I) : invalid")

        if robot_posx is not None:
            lines.append(
                f"TCP posx    : X={robot_posx[0]:.1f}, Y={robot_posx[1]:.1f}, Z={robot_posx[2]:.1f}, "
                f"RX={robot_posx[3]:.1f}, RY={robot_posx[4]:.1f}, RZ={robot_posx[5]:.1f}"
            )
        else:
            lines.append("TCP posx    : invalid")

        y = 35
        for line in lines:
            cv2.putText(
                heatmap,
                line,
                (20, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2,
                cv2.LINE_AA
            )
            y += 30

        if filtered_depth is not None:
            label = f"{filtered_depth:.0f} mm"
        else:
            label = "invalid"

        lx = min(u + 12, w - 180)
        ly = max(v - 10, 20)
        cv2.putText(
            heatmap,
            label,
            (lx, ly),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
            cv2.LINE_AA
        )

    def process_center_point(self):
        if self.depth_image is None:
            self.get_logger().warn("No depth image yet.")
            return

        if None in [self.fx, self.fy, self.cx, self.cy]:
            self.get_logger().warn("No camera intrinsics yet.")
            return

        filtered_depth, u, v, patch = self.get_center_filtered_depth()
        heatmap = self.make_heatmap(self.depth_image)

        camera_point = None
        base_point_direct = None
        base_point_inverse = None
        robot_posx = None

        if filtered_depth is not None:
            camera_point = self.pixel_to_camera_3d(
                u=u,
                v=v,
                depth_mm=filtered_depth,
                fx=self.fx,
                fy=self.fy,
                cx=self.cx,
                cy=self.cy
            )

            try:
                robot_posx = get_current_posx()[0]

                if os.path.exists(self.gripper2cam_path):
                    try:
                        base_point_direct = self.transform_camera_to_base_direct(
                            camera_coords=camera_point,
                            gripper2cam_path=self.gripper2cam_path,
                            robot_pos=robot_posx
                        )
                    except Exception as e:
                        self.get_logger().warn(f"Direct transform failed: {e}")

                    try:
                        base_point_inverse = self.transform_camera_to_base_inverse(
                            camera_coords=camera_point,
                            gripper2cam_path=self.gripper2cam_path,
                            robot_pos=robot_posx
                        )
                    except Exception as e:
                        self.get_logger().warn(f"Inverse transform failed: {e}")
                else:
                    self.get_logger().warn(f"Transform file not found: {self.gripper2cam_path}")

            except Exception as e:
                self.get_logger().warn(f"Failed to get current robot pose or transform: {e}")

        self.last_filtered_depth = filtered_depth
        self.last_camera_point = camera_point
        self.last_base_point_direct = base_point_direct
        self.last_base_point_inverse = base_point_inverse
        self.last_robot_posx = robot_posx

        self.draw_overlay(
            heatmap=heatmap,
            u=u,
            v=v,
            filtered_depth=filtered_depth,
            camera_point=camera_point,
            base_point_direct=base_point_direct,
            base_point_inverse=base_point_inverse,
            robot_posx=robot_posx
        )

        cv2.imshow(self.window_name, heatmap)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC
            self.get_logger().info("ESC pressed. Shutting down...")
            rclpy.shutdown()

        log_msg = f"center=({u},{v})"
        if filtered_depth is not None:
            log_msg += f", depth={filtered_depth:.1f} mm"
        if camera_point is not None:
            log_msg += f", camera={camera_point.tolist()}"
        if base_point_direct is not None:
            log_msg += f", base_direct={base_point_direct.tolist()}"
        if base_point_inverse is not None:
            log_msg += f", base_inverse={base_point_inverse.tolist()}"
        self.get_logger().info(log_msg)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    node = CenterDepthToBaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()