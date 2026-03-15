#!/usr/bin/env python3
import time
import cv2
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
        # debug visualize params
        # -----------------------------
        self.node.declare_parameter("debug_show_image", False)
        self.node.declare_parameter("debug_window_name", "wall_tile_debug")
        self.node.declare_parameter("debug_wait_ms", 1)

        # wall ROI (그리기 전용)
        self.node.declare_parameter("wall_roi_center_u", 422.5)
        self.node.declare_parameter("wall_roi_center_v", 223.5)
        self.node.declare_parameter("wall_roi_w", 349.0)
        self.node.declare_parameter("wall_roi_h", 273.0)
        self.node.declare_parameter("wall_roi_yaw_deg", 0.00851928332822126)

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

        self.debug_show_image = bool(self.node.get_parameter("debug_show_image").value)
        self.debug_window_name = str(self.node.get_parameter("debug_window_name").value)
        self.debug_wait_ms = int(self.node.get_parameter("debug_wait_ms").value)

        self.wall_roi_center_u = float(self.node.get_parameter("wall_roi_center_u").value)
        self.wall_roi_center_v = float(self.node.get_parameter("wall_roi_center_v").value)
        self.wall_roi_w = float(self.node.get_parameter("wall_roi_w").value)
        self.wall_roi_h = float(self.node.get_parameter("wall_roi_h").value)
        self.wall_roi_yaw_deg = float(self.node.get_parameter("wall_roi_yaw_deg").value)

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
        return self.localizer.depth_image.shape[:2]

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
    # debug visualization
    # --------------------------------------------------
    def _get_color_image_for_debug(self):
        """
        DepthLocalizer 내부 color 이미지 이름 후보를 순서대로 확인.
        없으면 None 반환.
        """
        candidates = [
            "color_image",
            "rgb_image",
            "yolo_image",
            "image",
            "latest_color_image",
        ]

        for name in candidates:
            if hasattr(self.localizer, name):
                img = getattr(self.localizer, name)
                if img is not None:
                    return img.copy()

        return None

    def _get_depth_vis_for_debug(self):
        if self.localizer.depth_image is None:
            return None

        depth = self.localizer.depth_image.astype(np.float32)
        valid = (depth >= self.min_mm) & (depth <= self.max_mm)

        if not np.any(valid):
            vis = np.zeros(depth.shape, dtype=np.uint8)
            return cv2.cvtColor(vis, cv2.COLOR_GRAY2BGR)

        vis = np.zeros(depth.shape, dtype=np.uint8)
        dmin = float(np.min(depth[valid]))
        dmax = float(np.max(depth[valid]))

        if dmax - dmin > 1e-6:
            vis[valid] = np.clip(
                (depth[valid] - dmin) / (dmax - dmin) * 255.0,
                0,
                255,
            ).astype(np.uint8)

        vis = cv2.applyColorMap(vis, cv2.COLORMAP_JET)
        vis[~valid] = (0, 0, 0)
        return vis

    def _draw_rotated_roi(self, img, cx, cy, w, h, theta_rad, color=(0, 255, 0), label=None):
        rect = ((float(cx), float(cy)), (float(w), float(h)), float(np.degrees(theta_rad)))
        box = cv2.boxPoints(rect)
        box = np.int32(np.round(box))
        cv2.polylines(img, [box], True, color, 2, lineType=cv2.LINE_AA)

        center_pt = (int(round(cx)), int(round(cy)))
        cv2.circle(img, center_pt, 3, color, -1, lineType=cv2.LINE_AA)

        if label:
            tx = int(np.min(box[:, 0]))
            ty = int(np.min(box[:, 1])) - 8
            ty = max(18, ty)
            cv2.putText(
                img,
                label,
                (tx, ty),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
                cv2.LINE_AA,
            )

    def _draw_points(self, img, pts, color=(255, 255, 255)):
        for (u, v) in pts:
            cv2.circle(img, (int(u), int(v)), 2, color, -1, lineType=cv2.LINE_AA)

    def _crop_rotated_roi(self, img, cx, cy, w, h, theta_rad):
        if img is None:
            return None

        h_img, w_img = img.shape[:2]
        M = cv2.getRotationMatrix2D((float(cx), float(cy)), float(np.degrees(theta_rad)), 1.0)
        rotated = cv2.warpAffine(img, M, (w_img, h_img))

        x1 = int(round(cx - w / 2.0))
        y1 = int(round(cy - h / 2.0))
        x2 = int(round(cx + w / 2.0))
        y2 = int(round(cy + h / 2.0))

        x1 = max(0, min(w_img - 1, x1))
        y1 = max(0, min(h_img - 1, y1))
        x2 = max(0, min(w_img, x2))
        y2 = max(0, min(h_img, y2))

        if x2 <= x1 or y2 <= y1:
            return None

        return rotated[y1:y2, x1:x2].copy()

    def _fit_debug_panel(self, img, out_w, out_h):
        if img is None:
            return np.zeros((out_h, out_w, 3), dtype=np.uint8)

        if img.ndim == 2:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        h, w = img.shape[:2]
        if h <= 0 or w <= 0:
            return np.zeros((out_h, out_w, 3), dtype=np.uint8)

        scale = min(out_w / w, out_h / h)
        new_w = max(1, int(w * scale))
        new_h = max(1, int(h * scale))
        resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_AREA)

        canvas = np.zeros((out_h, out_w, 3), dtype=np.uint8)
        x0 = (out_w - new_w) // 2
        y0 = (out_h - new_h) // 2
        canvas[y0:y0 + new_h, x0:x0 + new_w] = resized
        return canvas

    def _put_panel_title(self, img, title):
        out = img.copy()
        cv2.rectangle(out, (0, 0), (out.shape[1], 28), (25, 25, 25), -1)
        cv2.putText(
            out,
            title,
            (10, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return out

    def _show_debug_image(self):
        color_img = self._get_color_image_for_debug()
        depth_vis = self._get_depth_vis_for_debug()

        if color_img is None or depth_vis is None:
            return

        color_draw = color_img.copy()
        depth_draw = depth_vis.copy()

        # tile ROI + sample points + depth crop
        tile_crop_rows = []
        if self.latest_tile_array is not None:
            for i, tile in enumerate(self.latest_tile_array.tiles):
                cx = float(tile.pose.x)
                cy = float(tile.pose.y)
                w = float(tile.width)
                h = float(tile.height)
                theta_rad = float(tile.pose.theta)

                name = tile.pattern_name.strip() if tile.pattern_name.strip() else f"tile_{i+1}"

                self._draw_rotated_roi(
                    color_draw, cx, cy, w, h, theta_rad,
                    color=(0, 255, 0), label=name
                )
                self._draw_rotated_roi(
                    depth_draw, cx, cy, w, h, theta_rad,
                    color=(0, 255, 0), label=name
                )

                uv_points = self.sample_points_in_rotated_roi(
                    cx=cx,
                    cy=cy,
                    w=w,
                    h=h,
                    theta_rad=theta_rad,
                    n_points=self.sample_n_points,
                    shrink_px=self.tile_shrink_px_depth,
                )
                self._draw_points(depth_draw, uv_points, color=(255, 255, 255))

                tile_crop = self._crop_rotated_roi(depth_vis, cx, cy, w, h, theta_rad)
                tile_crop = self._fit_debug_panel(tile_crop, 220, 160)
                tile_crop = self._put_panel_title(tile_crop, name)
                tile_crop_rows.append(tile_crop)

        # wall ROI는 그리기만 하고 crop은 안 함
        wall_theta_rad = np.radians(self.wall_roi_yaw_deg)

        self._draw_rotated_roi(
            color_draw,
            self.wall_roi_center_u,
            self.wall_roi_center_v,
            self.wall_roi_w,
            self.wall_roi_h,
            wall_theta_rad,
            color=(0, 255, 255),
            label="wall_roi",
        )
        self._draw_rotated_roi(
            depth_draw,
            self.wall_roi_center_u,
            self.wall_roi_center_v,
            self.wall_roi_w,
            self.wall_roi_h,
            wall_theta_rad,
            color=(0, 255, 255),
            label="wall_roi",
        )

        panel1 = self._put_panel_title(self._fit_debug_panel(color_draw, 640, 480), "YOLO / Color")
        panel2 = self._put_panel_title(self._fit_debug_panel(depth_vis, 640, 480), "Raw Depth")
        panel3 = self._put_panel_title(self._fit_debug_panel(depth_draw, 640, 480), "Depth + Tile ROI + Wall ROI")

        top = np.hstack([panel1, panel2])

        if len(tile_crop_rows) == 0:
            crops = np.zeros((480, 640, 3), dtype=np.uint8)
            crops = self._put_panel_title(crops, "Tile ROI crops")
        else:
            rows = []
            row = []
            blank = np.zeros((160, 220, 3), dtype=np.uint8)

            for i, crop in enumerate(tile_crop_rows):
                row.append(crop)
                if len(row) == 2 or i == len(tile_crop_rows) - 1:
                    while len(row) < 2:
                        row.append(blank.copy())
                    rows.append(np.hstack(row))
                    row = []

            crops = rows[0]
            for r in rows[1:]:
                crops = np.vstack([crops, r])

            crops = self._fit_debug_panel(crops, 640, 480)
            crops = self._put_panel_title(crops, "Tile ROI crops")

        bottom = np.hstack([panel3, crops])
        final_img = np.vstack([top, bottom])

        cv2.imshow(self.debug_window_name, final_img)
        cv2.waitKey(self.debug_wait_ms)

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
    def analyze_once(self, robot_posx, wait_timeout_sec=None, debug_show=None):
        wait_timeout_sec = self.wait_timeout_sec if wait_timeout_sec is None else float(wait_timeout_sec)

        if debug_show is None:
            debug_show = self.debug_show_image

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

        if debug_show:
            try:
                self._show_debug_image()
            except Exception as e:
                self.node.get_logger().warn(f"[InspectionEngine] debug image show failed: {e}")

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