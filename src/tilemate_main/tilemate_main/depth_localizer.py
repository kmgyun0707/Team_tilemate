#!/usr/bin/env python3
import os
import numpy as np
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Image, CameraInfo


class DepthLocalizer:
    def __init__(self, node, gripper2cam_path, use_inverse=True):
        """
        Args:
            node: rclpy Node
            gripper2cam_path: T_gripper2camera.npy 경로
            use_inverse:
                True  -> base = T_base_gripper @ inv(T_gripper_camera) @ P_camera
                False -> base = T_base_gripper @ T_gripper_camera @ P_camera
        """
        self.node = node
        self.depth_image = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.gripper2cam_path = gripper2cam_path
        self.use_inverse = use_inverse

        self.node.create_subscription(
            Image,
            "/camera/camera/depth/image_rect_raw",
            self.depth_callback,
            10
        )
        self.node.create_subscription(
            CameraInfo,
            "/camera/camera/depth/camera_info",
            self.camera_info_callback,
            10
        )

        self.node.get_logger().info(
            f"[DepthLocalizer] init done, gripper2cam_path={self.gripper2cam_path}, "
            f"use_inverse={self.use_inverse}"
        )

    # --------------------------------------------------
    # ROS callbacks
    # --------------------------------------------------

    def depth_callback(self, msg: Image):
        if msg.encoding != "16UC1":
            self.node.get_logger().error(
                f"[DepthLocalizer] Unsupported depth encoding: {msg.encoding}"
            )
            return

        try:
            self.depth_image = np.frombuffer(
                msg.data, dtype=np.uint16
            ).reshape(msg.height, msg.width)
        except Exception as e:
            self.node.get_logger().error(f"[DepthLocalizer] Depth reshape failed: {e}")
            self.depth_image = None

    def camera_info_callback(self, msg: CameraInfo):
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])

    # --------------------------------------------------
    # status helpers
    # --------------------------------------------------

    def is_depth_ready(self):
        return self.depth_image is not None

    def is_camera_info_ready(self):
        return None not in [self.fx, self.fy, self.cx, self.cy]

    def is_ready(self):
        return self.is_depth_ready() and self.is_camera_info_ready()

    # --------------------------------------------------
    # math helpers
    # --------------------------------------------------

    def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
        """
        robot posx -> 4x4 homogeneous matrix
        """
        R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T

    def pixel_to_camera_3d(self, u, v, depth_mm):
        """
        pixel (u, v) + depth(mm) -> camera 3D point(mm)
        """
        if not self.is_camera_info_ready():
            raise RuntimeError("Camera intrinsics are not ready")

        x = (float(u) - self.cx) * float(depth_mm) / self.fx
        y = (float(v) - self.cy) * float(depth_mm) / self.fy
        z = float(depth_mm)

        return np.array([x, y, z], dtype=np.float64)

    def transform_camera_to_base(self, camera_point, robot_posx):
        """
        camera_point(mm) -> base_point(mm)

        robot_posx: [x, y, z, rx, ry, rz]
        """
        if not os.path.exists(self.gripper2cam_path):
            raise FileNotFoundError(
                f"T_gripper2camera.npy not found: {self.gripper2cam_path}"
            )

        T_gripper_camera = np.load(self.gripper2cam_path)

        if T_gripper_camera.shape != (4, 4):
            raise ValueError(
                f"Transform matrix shape must be (4,4), got {T_gripper_camera.shape}"
            )

        if len(robot_posx) < 6:
            raise ValueError(f"robot_posx must have 6 elements, got {robot_posx}")

        x, y, z, rx, ry, rz = robot_posx[:6]
        T_base_gripper = self.get_robot_pose_matrix(x, y, z, rx, ry, rz)

        P_camera = np.append(np.array(camera_point, dtype=np.float64), 1.0)

        if self.use_inverse:
            T_camera_gripper = np.linalg.inv(T_gripper_camera)
            P_base = T_base_gripper @ T_camera_gripper @ P_camera
        else:
            P_base = T_base_gripper @ T_gripper_camera @ P_camera

        return P_base[:3]

    # --------------------------------------------------
    # depth filtering
    # --------------------------------------------------

    def get_filtered_depth_at(self, u, v, kernel_size=5, min_mm=200, max_mm=5000, inlier_thresh_mm=100):
        """
        (u, v) 중심 kernel_size x kernel_size 영역에서 filtered depth(mm) 반환
        """
        if self.depth_image is None:
            return None

        if kernel_size % 2 == 0:
            raise ValueError("kernel_size must be odd")

        h, w = self.depth_image.shape
        half = kernel_size // 2

        x1 = max(0, int(u) - half)
        x2 = min(w, int(u) + half + 1)
        y1 = max(0, int(v) - half)
        y2 = min(h, int(v) + half + 1)

        patch = self.depth_image[y1:y2, x1:x2]

        valid = patch[(patch > min_mm) & (patch < max_mm)]

        if valid.size == 0:
            return None

        median_depth = np.median(valid)
        inliers = valid[np.abs(valid - median_depth) <= inlier_thresh_mm]

        if inliers.size > 0:
            filtered_depth = float(np.median(inliers))
        else:
            filtered_depth = float(median_depth)

        return filtered_depth

    def get_center_filtered_depth(self, kernel_size=5, min_mm=200, max_mm=5000, inlier_thresh_mm=100):
        """
        중심점 (cx_img, cy_img)의 filtered depth(mm), u, v 반환
        Returns:
            depth_mm, u, v
            실패 시: (None, u, v) 또는 (None, None, None)
        """
        if self.depth_image is None:
            return None, None, None

        h, w = self.depth_image.shape
        u = w // 2
        v = h // 2

        depth_mm = self.get_filtered_depth_at(
            u=u,
            v=v,
            kernel_size=kernel_size,
            min_mm=min_mm,
            max_mm=max_mm,
            inlier_thresh_mm=inlier_thresh_mm,
        )

        return depth_mm, u, v

    # --------------------------------------------------
    # high-level API
    # --------------------------------------------------

    def estimate_pick_base_point_from_pixel(self, u, v, robot_posx,
                                            kernel_size=5,
                                            min_mm=200,
                                            max_mm=5000,
                                            inlier_thresh_mm=100,
                                            z_offset_mm=0.0):
        """
        임의 픽셀 (u, v)의 base 좌표계 3D 점 추정
        """
        if not self.is_ready():
            self.node.get_logger().warn("[DepthLocalizer] depth/camera_info not ready")
            return None

        depth_mm = self.get_filtered_depth_at(
            u=u,
            v=v,
            kernel_size=kernel_size,
            min_mm=min_mm,
            max_mm=max_mm,
            inlier_thresh_mm=inlier_thresh_mm,
        )

        if depth_mm is None:
            self.node.get_logger().warn(
                f"[DepthLocalizer] No valid depth at pixel ({u}, {v})"
            )
            return None

        camera_point = self.pixel_to_camera_3d(u, v, depth_mm)
        base_point = self.transform_camera_to_base(camera_point, robot_posx)

        base_point = np.array(base_point, dtype=np.float64)
        base_point[2] += float(z_offset_mm)

        self.node.get_logger().info(
            f"[DepthLocalizer] pixel=({u},{v}), depth={depth_mm:.1f} mm, "
            f"camera={camera_point.tolist()}, base={base_point.tolist()}"
        )

        return base_point

    def estimate_center_pick_base_point(self, robot_posx,
                                        kernel_size=5,
                                        min_mm=200,
                                        max_mm=5000,
                                        inlier_thresh_mm=100,
                                        z_offset_mm=0.0):
        """
        화면 중심점 기준으로 base 좌표계 점 반환
        Returns:
            np.array([x, y, z]) or None
        """
        if not self.is_ready():
            self.node.get_logger().warn("[DepthLocalizer] depth/camera_info not ready")
            return None

        depth_mm, u, v = self.get_center_filtered_depth(
            kernel_size=kernel_size,
            min_mm=min_mm,
            max_mm=max_mm,
            inlier_thresh_mm=inlier_thresh_mm,
        )

        if depth_mm is None:
            self.node.get_logger().warn("[DepthLocalizer] No valid depth in center patch")
            return None

        camera_point = self.pixel_to_camera_3d(u, v, depth_mm)
        base_point = self.transform_camera_to_base(camera_point, robot_posx)

        base_point = np.array(base_point, dtype=np.float64)
        base_point[0] -= 30.0
        base_point[1] += 15.0
        base_point[2] += float(z_offset_mm)

        self.node.get_logger().info(
            f"[DepthLocalizer] center=({u},{v}), depth={depth_mm:.1f} mm, "
            f"camera={camera_point.tolist()}, base={base_point.tolist()}"
        )

        return base_point