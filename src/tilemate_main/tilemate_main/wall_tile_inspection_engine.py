#!/usr/bin/env python3
import time
import numpy as np
import rclpy

from custom_tile_msgs.msg import TileArray
from tilemate_main.depth_localizer import DepthLocalizer


class WallTileInspectionEngine:
    def __init__(self, node):
        self.node = node

        # -----------------------------
        # params
        # -----------------------------
        self.node.declare_parameter("gripper2cam_path", "")
        self.node.declare_parameter("use_inverse", False)

        self.node.declare_parameter("min_mm", 200)
        self.node.declare_parameter("max_mm", 3000)
        self.node.declare_parameter("wait_timeout_sec", 5.0)

        self.node.declare_parameter("sample_n_points", 25)
        self.node.declare_parameter("depth_kernel_size", 5)
        self.node.declare_parameter("depth_inlier_thresh_mm", 80.0)

        # YOLO ROI를 안쪽으로 줄여서 사용할 픽셀 수
        self.node.declare_parameter("tile_shrink_px_depth", 10)

        # -----------------------------
        # cached params
        # -----------------------------
        gripper2cam_path = self.node.get_parameter("gripper2cam_path").value
        use_inverse = self.node.get_parameter("use_inverse").value

        self.min_mm = int(self.node.get_parameter("min_mm").value)
        self.max_mm = int(self.node.get_parameter("max_mm").value)
        self.wait_timeout_sec = float(self.node.get_parameter("wait_timeout_sec").value)
        self.sample_n_points = int(self.node.get_parameter("sample_n_points").value)
        self.depth_kernel_size = int(self.node.get_parameter("depth_kernel_size").value)
        self.depth_inlier_thresh_mm = float(self.node.get_parameter("depth_inlier_thresh_mm").value)
        self.tile_shrink_px_depth = int(self.node.get_parameter("tile_shrink_px_depth").value)

        # -----------------------------
        # localizer
        # -----------------------------
        self.localizer = DepthLocalizer(
            node=self.node,
            gripper2cam_path=gripper2cam_path,
            use_inverse=use_inverse,
        )

        # -----------------------------
        # latest yolo tile array
        # -----------------------------
        self.latest_tile_array = None
        self.latest_tile_stamp = None

        self.tile_array_sub = self.node.create_subscription(
            TileArray,
            "/yolo/tile_array",
            self.tile_array_callback,
            10,
        )

    # --------------------------------------------------
    # callbacks / waits
    # --------------------------------------------------
    def tile_array_callback(self, msg: TileArray):
        self.latest_tile_array = msg
        self.latest_tile_stamp = time.time()

    def wait_until_ready(self, timeout_sec=None):
        timeout_sec = self.wait_timeout_sec if timeout_sec is None else float(timeout_sec)
        start = time.time()

        while self.node.context.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)

            if self.localizer.is_ready():
                return True

            if time.time() - start > timeout_sec:
                return False

        return False

    def wait_for_tile_array(self, timeout_sec=5.0, max_age_sec=0.5):
        start = time.time()

        while self.node.context.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)

            if (
                self.latest_tile_array is not None
                and len(self.latest_tile_array.tiles) > 0
                and self.latest_tile_stamp is not None
                and (time.time() - self.latest_tile_stamp) <= max_age_sec
            ):
                return True

            if time.time() - start > timeout_sec:
                return False

        return False

    # --------------------------------------------------
    # helpers
    # --------------------------------------------------
    def _depth_hw(self):
        if self.localizer.depth_image is None:
            raise RuntimeError("depth_image is None")
        return self.localizer.depth_image.shape

    def normal_and_theta_to_rpy_deg(self, normal, theta_rad):
        nx, ny, nz = [float(v) for v in normal]

        roll_deg = float(np.degrees(np.arctan2(ny, nz)))
        pitch_deg = float(np.degrees(np.arctan2(-nx, np.sqrt(ny * ny + nz * nz))))
        yaw_deg = float(np.degrees(theta_rad))

        return roll_deg, pitch_deg, yaw_deg

    # --------------------------------------------------
    # ROI sampling
    # --------------------------------------------------
    def sample_points_in_rotated_roi(self, cx, cy, w, h, theta_rad, n_points=25, shrink_px=0):
        h_img, w_img = self._depth_hw()

        inner_w = max(2.0, float(w) - 2.0 * float(shrink_px))
        inner_h = max(2.0, float(h) - 2.0 * float(shrink_px))

        grid_n = int(np.ceil(np.sqrt(max(4, n_points))))
        xs = np.linspace(-0.5 * inner_w, 0.5 * inner_w, grid_n)
        ys = np.linspace(-0.5 * inner_h, 0.5 * inner_h, grid_n)

        c = np.cos(theta_rad)
        s = np.sin(theta_rad)

        pts = []
        seen = set()

        for yy in ys:
            for xx in xs:
                u = cx + xx * c - yy * s
                v = cy + xx * s + yy * c

                ui = int(round(u))
                vi = int(round(v))

                if not (0 <= ui < w_img and 0 <= vi < h_img):
                    continue

                p = (ui, vi)
                if p not in seen:
                    seen.add(p)
                    pts.append(p)

        if len(pts) > n_points:
            idx = np.linspace(0, len(pts) - 1, n_points).astype(int)
            pts = [pts[i] for i in idx]

        return pts

    # --------------------------------------------------
    # depth -> 3D
    # --------------------------------------------------
    def uv_to_3d_points(self, uv_points):
        points_xyz = []

        for (u, v) in uv_points:
            depth_mm = self.localizer.get_filtered_depth_at(
                u=int(u),
                v=int(v),
                kernel_size=self.depth_kernel_size,
                min_mm=self.min_mm,
                max_mm=self.max_mm,
                inlier_thresh_mm=self.depth_inlier_thresh_mm,
            )
            if depth_mm is None:
                continue

            p = self.localizer.pixel_to_camera_3d(int(u), int(v), depth_mm)
            if p is not None:
                points_xyz.append(p)

        if len(points_xyz) < 3:
            return None

        return np.asarray(points_xyz, dtype=np.float64)

    # --------------------------------------------------
    # plane fitting
    # --------------------------------------------------
    def fit_plane_svd(self, points_xyz):
        if points_xyz is None or len(points_xyz) < 3:
            return None

        pts = np.asarray(points_xyz, dtype=np.float64)
        centroid = np.mean(pts, axis=0)
        centered = pts - centroid

        _, _, vh = np.linalg.svd(centered, full_matrices=False)
        normal = vh[-1, :]
        norm = np.linalg.norm(normal)
        if norm < 1e-12:
            return None

        normal = normal / norm
        d = -float(np.dot(normal, centroid))

        if normal[2] < 0:
            normal = -normal
            d = -d

        return normal, d, centroid

    # --------------------------------------------------
    # base center estimation
    # --------------------------------------------------
    def estimate_tile_center_base_point(self, cx, cy, robot_posx):
        return self.localizer.estimate_pick_base_point_from_pixel(
            u=int(round(cx)),
            v=int(round(cy)),
            robot_posx=robot_posx,
            kernel_size=self.depth_kernel_size,
            min_mm=self.min_mm,
            max_mm=self.max_mm,
            inlier_thresh_mm=self.depth_inlier_thresh_mm,
            z_offset_mm=0.0,
        )

    # --------------------------------------------------
    # one tile analysis
    # --------------------------------------------------
    def analyze_one_tile(self, tile, robot_posx, index=0):
        cx = float(tile.pose.x)
        cy = float(tile.pose.y)
        w = float(tile.width)
        h = float(tile.height)
        theta_rad = float(tile.pose.theta)

        uv_points = self.sample_points_in_rotated_roi(
            cx=cx,
            cy=cy,
            w=w,
            h=h,
            theta_rad=theta_rad,
            n_points=self.sample_n_points,
            shrink_px=self.tile_shrink_px_depth,
        )

        points_xyz = self.uv_to_3d_points(uv_points)
        if points_xyz is None:
            return None

        plane = self.fit_plane_svd(points_xyz)
        if plane is None:
            return None

        normal, d, centroid = plane
        roll_deg, pitch_deg, yaw_deg = self.normal_and_theta_to_rpy_deg(normal, theta_rad)

        base_center = self.estimate_tile_center_base_point(cx, cy, robot_posx)
        if base_center is None:
            self.node.get_logger().warn(
                f"[InspectionEngine] failed to estimate base center for tile[{index}]"
            )

        name = tile.pattern_name.strip() if tile.pattern_name.strip() else f"tile_{index + 1}"

        return {
            "name": name,
            "conf_score": float(tile.conf_score),
            "center_uv": [cx, cy],
            "size_uv": [w, h],
            "rpy_deg": [float(roll_deg), float(pitch_deg), float(yaw_deg)],
            "plane_normal": [float(normal[0]), float(normal[1]), float(normal[2])],
            "plane_d": float(d),
            "plane_centroid_mm": [float(centroid[0]), float(centroid[1]), float(centroid[2])],
            "base_center_mm": (
                [float(base_center[0]), float(base_center[1]), float(base_center[2])]
                if base_center is not None else None
            ),
        }

    # --------------------------------------------------
    # all tiles
    # --------------------------------------------------
    def analyze_once(self, robot_posx, wait_timeout_sec=None):
        wait_timeout_sec = self.wait_timeout_sec if wait_timeout_sec is None else float(wait_timeout_sec)

        if robot_posx is None or len(robot_posx) < 6:
            return {
                "success": False,
                "message": "invalid_robot_posx",
                "result_dict": None,
            }

        if not self.wait_until_ready(timeout_sec=wait_timeout_sec):
            return {
                "success": False,
                "message": "sensor_not_ready",
                "result_dict": None,
            }

        if not self.wait_for_tile_array(timeout_sec=5.0):
            return {
                "success": False,
                "message": "tile_array_not_ready",
                "result_dict": None,
            }

        results = []
        for i, tile in enumerate(self.latest_tile_array.tiles):
            try:
                item = self.analyze_one_tile(tile, robot_posx, i)
                if item is not None:
                    results.append(item)
            except Exception as e:
                self.node.get_logger().warn(f"[InspectionEngine] tile[{i}] analysis failed: {e}")

        if len(results) == 0:
            return {
                "success": False,
                "message": "tile_analysis_failed",
                "result_dict": None,
            }

        payload = {
            "success": True,
            "message": "inspect_complete",
            "timestamp_sec": float(time.time()),
            "tiles": results,
        }

        return {
            "success": True,
            "message": "inspect_complete",
            "result_dict": payload,
        }