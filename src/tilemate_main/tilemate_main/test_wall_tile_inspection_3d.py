#!/usr/bin/env python3
import time
import json
import asyncio
import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from custom_tile_msgs.msg import TileArray

from tilemate_main.depth_localizer import DepthLocalizer
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class WallTileInspection3DNode(Node):
    def __init__(self):
        super().__init__("wall_tile_inspection_3d_node")

        # --------------------------------------------------
        # parameters
        # --------------------------------------------------
        self.declare_parameter("gripper2cam_path", "")
        self.declare_parameter("use_inverse", True)

        self.declare_parameter("min_mm", 200)
        self.declare_parameter("max_mm", 3000)
        self.declare_parameter("wait_timeout_sec", 5.0)

        self.declare_parameter("sample_n_points", 50)
        self.declare_parameter("sample_margin_px", 10)
        self.declare_parameter("depth_kernel_size", 5)
        self.declare_parameter("depth_inlier_thresh_mm", 80.0)

        self.declare_parameter("residual_clim_mm", 15.0)
        self.declare_parameter("residual_stride", 2)

        # RGB -> Depth scaling
        self.declare_parameter("rgb_width", 640)
        self.declare_parameter("rgb_height", 360)
        self.declare_parameter("depth_width", 848)
        self.declare_parameter("depth_height", 480)

        # wall ROI: depth 기준 고정 ROI
        self.declare_parameter("wall_roi_depth", [200, 50, 650, 400])

        # yolo roi margin
        self.declare_parameter("tile_margin_px_depth", -10)

        self.window_name = "Wall/Tile ROI Preview (Auto from YOLO)"
        self.result_window_name = "Wall Tile Inspection 3D Result"

        gripper2cam_path = self.get_parameter("gripper2cam_path").get_parameter_value().string_value
        use_inverse = self.get_parameter("use_inverse").get_parameter_value().bool_value

        self.rgb_width = int(self.get_parameter("rgb_width").value)
        self.rgb_height = int(self.get_parameter("rgb_height").value)
        self.depth_width = int(self.get_parameter("depth_width").value)
        self.depth_height = int(self.get_parameter("depth_height").value)

        # self.scale_x = float(self.depth_width) / float(self.rgb_width)
        # self.scale_y = float(self.depth_height) / float(self.rgb_height)
        self.scale_x = 1.0
        self.scale_y = 1.0
        self.wall_roi = tuple(int(v) for v in self.get_parameter("wall_roi_depth").value)
        self.tile_margin_px_depth = int(self.get_parameter("tile_margin_px_depth").value)

        self.localizer = DepthLocalizer(
            node=self,
            gripper2cam_path=gripper2cam_path,
            use_inverse=use_inverse
        )
        self.bridge = CvBridge()
        self.latest_annotated_image = None
        self.latest_annotated_stamp = None

        self.annotated_image_sub = self.create_subscription(
            Image,
            "/yolo/annotated_image",
            self.annotated_image_callback,
            10,
        )
        self.latest_tile_array = None
        self.latest_tile_stamp = None
        self.tile_regions = []

        self.tile_array_sub = self.create_subscription(
            TileArray,
            "/yolo/tile_array",
            self.tile_array_callback,
            10,
        )

        self.get_logger().info(
            f"[WallTileInspection3DNode] RGB({self.rgb_width}x{self.rgb_height}) "
            f"-> DEPTH({self.depth_width}x{self.depth_height}) "
            f"scale_x={self.scale_x:.4f}, scale_y={self.scale_y:.4f}"
        )
        self.get_logger().info(f"[WallTileInspection3DNode] wall_roi(depth)={self.wall_roi}")
        self.get_logger().info("[WallTileInspection3DNode] initialized")

    # --------------------------------------------------
    # ready / callbacks
    # --------------------------------------------------
    def wait_until_ready(self, timeout_sec=5.0):
        start = time.time()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.localizer.is_ready():
                self.get_logger().info("[WallTileInspection3DNode] depth and camera_info ready")
                return True
            if time.time() - start > timeout_sec:
                self.get_logger().warn("[WallTileInspection3DNode] timeout waiting for depth/camera_info")
                return False
        return False
    def annotated_image_callback(self, msg: Image):
        try:
            self.latest_annotated_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.latest_annotated_stamp = time.time()
        except Exception as e:
            self.get_logger().warn(f"[WallTileInspection3DNode] annotated_image_callback failed: {e}")
    def tile_array_callback(self, msg: TileArray):
        self.latest_tile_array = msg
        self.latest_tile_stamp = time.time()

    def wait_for_tile_array(self, timeout_sec=5.0):
        start = time.time()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_tile_array is not None and len(self.latest_tile_array.tiles) > 0:
                self.get_logger().info(
                    f"[WallTileInspection3DNode] yolo tile array ready: {len(self.latest_tile_array.tiles)} tiles"
                )
                return True
            if time.time() - start > timeout_sec:
                self.get_logger().warn("[WallTileInspection3DNode] timeout waiting for /yolo/tile_array")
                return False
        return False

    # --------------------------------------------------
    # coordinate helpers
    # --------------------------------------------------

    def yolo_image_callback(self, msg: Image):
        try:
            if msg.encoding.lower() in ["bgr8", "rgb8"]:
                img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            else:
                img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            self.latest_yolo_image = img
            self.latest_yolo_stamp = time.time()
        except Exception as e:
            self.get_logger().warn(f"[WallTileInspection3DNode] yolo_image_callback failed: {e}")
    def wait_for_yolo_image(self, timeout_sec=5.0):
        start = time.time()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_yolo_image is not None:
                self.get_logger().info("[WallTileInspection3DNode] yolo debug image ready")
                return True
            if time.time() - start > timeout_sec:
                self.get_logger().warn("[WallTileInspection3DNode] timeout waiting for yolo debug image")
                return False
        return False
    def compose_yolo_depth_side_by_side(self, wall_region, tile_regions, min_mm, max_mm, out_h=480):
        if self.latest_yolo_image is None or self.localizer.depth_image is None:
            return None

        # -----------------------------
        # left: YOLO detection overlay
        # -----------------------------
        yolo_vis = self.latest_yolo_image.copy()

        # YOLO 이미지 위에 중심점 정도만 추가로 찍어도 디버깅에 도움됨
        for item in tile_regions:
            cx_rgb = int(round(item["rgb_center"][0]))
            cy_rgb = int(round(item["rgb_center"][1]))
            cv2.circle(yolo_vis, (cx_rgb, cy_rgb), 4, (0, 255, 255), -1)
            cv2.putText(
                yolo_vis, item["name"],
                (cx_rgb + 5, max(20, cy_rgb - 5)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2
            )

        # -----------------------------
        # right: Depth map + depth ROI
        # -----------------------------
        depth_vis = self.depth_to_colormap(self.localizer.depth_image, min_mm, max_mm)

        self.draw_region(depth_vis, wall_region, (0, 255, 255), thickness=2, label="WALL")
        for item in tile_regions:
            self.draw_region(depth_vis, item, (0, 255, 0), thickness=2, label=item["name"])

            # bbox도 같이 그리면 변환 상태 디버깅 쉬움
            u1, v1, u2, v2 = item["roi"]
            cv2.rectangle(depth_vis, (u1, v1), (u2, v2), (0, 0, 255), 1)

        # -----------------------------
        # same display height
        # -----------------------------
        yh, yw = yolo_vis.shape[:2]
        dh, dw = depth_vis.shape[:2]

        yolo_w = int(round(yw * (out_h / float(yh))))
        depth_w = int(round(dw * (out_h / float(dh))))

        yolo_vis = cv2.resize(yolo_vis, (yolo_w, out_h))
        depth_vis = cv2.resize(depth_vis, (depth_w, out_h))

        # header text
        cv2.putText(
            yolo_vis, "YOLO Detection Result", (10, 28),
            cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2
        )
        cv2.putText(
            depth_vis, "Depth Map + Depth ROI", (10, 28),
            cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2
        )

        side = np.hstack([yolo_vis, depth_vis])
        return side
    def rgb_to_depth_uv(self, u_rgb, v_rgb):
        u_depth = float(u_rgb) * self.scale_x
        v_depth = float(v_rgb) * self.scale_y
        return u_depth, v_depth

    def rgb_wh_to_depth_wh(self, w_rgb, h_rgb):
        w_depth = float(w_rgb) * self.scale_x
        h_depth = float(h_rgb) * self.scale_y
        return w_depth, h_depth

    def clamp_point(self, u, v):
        if self.localizer.depth_image is None:
            raise RuntimeError("depth_image is None")
        h, w = self.localizer.depth_image.shape
        u = max(0.0, min(float(w - 1), float(u)))
        v = max(0.0, min(float(h - 1), float(v)))
        return u, v

    def clamp_roi(self, u1, v1, u2, v2):
        if self.localizer.depth_image is None:
            raise RuntimeError("depth_image is None")

        h, w = self.localizer.depth_image.shape
        u1 = max(0, min(w - 1, int(u1)))
        u2 = max(0, min(w, int(u2)))
        v1 = max(0, min(h - 1, int(v1)))
        v2 = max(0, min(h, int(v2)))

        if u2 <= u1 or v2 <= v1:
            raise ValueError(f"Invalid ROI: ({u1},{v1}) ~ ({u2},{v2})")

        return u1, v1, u2, v2

    def roi_center_uv(self, roi):
        u1, v1, u2, v2 = roi
        uc = int(round((u1 + u2) * 0.5))
        vc = int(round((v1 + v2) * 0.5))
        return uc, vc

    def mm_to_m_list(self, p):
        return [float(p[0]) / 1000.0, float(p[1]) / 1000.0, float(p[2]) / 1000.0]

    # --------------------------------------------------
    # rotated region helpers
    # --------------------------------------------------
    def create_wall_region(self):
        u1, v1, u2, v2 = self.clamp_roi(*self.wall_roi)
        cx = 0.5 * (u1 + u2)
        cy = 0.5 * (v1 + v2)
        w = float(u2 - u1)
        h = float(v2 - v1)

        return {
            "name": "wall",
            "type": "axis_aligned",
            "roi": (u1, v1, u2, v2),
            "center_uv": (cx, cy),
            "size_uv": (w, h),
            "theta_rad": 0.0,
            "theta_deg": 0.0,
            "conf_score": 1.0,
        }

    def create_tile_region_from_msg(self, tile, index, margin_px=0):
        cx_rgb = float(tile.pose.x)
        cy_rgb = float(tile.pose.y)
        w_rgb = float(tile.width)
        h_rgb = float(tile.height)
        theta_rad = float(tile.pose.theta)

        # 중심만 depth 좌표계로 이동
        cx_depth, cy_depth = self.rgb_to_depth_uv(cx_rgb, cy_rgb)

        # 크기는 rgb 기준 그대로 사용
        w_depth = float(w_rgb)
        h_depth = float(h_rgb)

        w_depth += 2.0 * float(margin_px)
        h_depth += 2.0 * float(margin_px)

        cx_depth, cy_depth = self.clamp_point(cx_depth, cy_depth)

        name = tile.pattern_name.strip() if tile.pattern_name.strip() else f"tile_{index + 1}"

        box = cv2.boxPoints(((cx_depth, cy_depth), (w_depth, h_depth), np.degrees(theta_rad)))
        bbox_u1 = int(np.floor(np.min(box[:, 0])))
        bbox_v1 = int(np.floor(np.min(box[:, 1])))
        bbox_u2 = int(np.ceil(np.max(box[:, 0])))
        bbox_v2 = int(np.ceil(np.max(box[:, 1])))
        bbox = self.clamp_roi(bbox_u1, bbox_v1, bbox_u2, bbox_v2)
        self.get_logger().info(
            f"[ROI DEBUG] rgb center=({cx_rgb:.2f},{cy_rgb:.2f}) size=({w_rgb:.2f},{h_rgb:.2f}) "
            f"-> depth center=({cx_depth:.2f},{cy_depth:.2f}) size=({w_depth:.2f},{h_depth:.2f})"
        )
        return {
            "name": name,
            "type": "rotated_rect",
            "roi": bbox,
            "center_uv": (cx_depth, cy_depth),
            "size_uv": (w_depth, h_depth),
            "theta_rad": theta_rad,
            "theta_deg": float(np.degrees(theta_rad)),
            "rgb_center": (cx_rgb, cy_rgb),
            "rgb_size": (w_rgb, h_rgb),
            "conf_score": float(tile.conf_score),
        }

    def build_tile_regions_from_yolo(self, margin_px=0):
        if self.latest_tile_array is None:
            return []

        regions = []
        for i, tile in enumerate(self.latest_tile_array.tiles):
            try:
                region = self.create_tile_region_from_msg(tile, i, margin_px=margin_px)
                regions.append(region)
            except Exception as e:
                self.get_logger().warn(
                    f"[WallTileInspection3DNode] failed to convert tile[{i}] to region: {e}"
                )
        return regions

    def get_region_box_points(self, region):
        cx, cy = region["center_uv"]
        w, h = region["size_uv"]
        angle_deg = region["theta_deg"]
        pts = cv2.boxPoints(((float(cx), float(cy)), (float(w), float(h)), float(angle_deg)))
        return np.asarray(pts, dtype=np.float32)

    def point_in_region(self, u, v, region):
        if region["type"] == "axis_aligned":
            u1, v1, u2, v2 = region["roi"]
            return (u1 <= u < u2) and (v1 <= v < v2)

        pts = self.get_region_box_points(region)
        inside = cv2.pointPolygonTest(pts, (float(u), float(v)), False)
        return inside >= 0

    def draw_region(self, image, region, color, thickness=2, label=None):
        label = region["name"] if label is None else label

        if region["type"] == "axis_aligned":
            u1, v1, u2, v2 = region["roi"]
            cv2.rectangle(image, (u1, v1), (u2, v2), color, thickness)
            cv2.putText(
                image, label, (u1, max(20, v1 - 5)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2
            )
            return

        pts = self.get_region_box_points(region).astype(np.int32)
        cv2.polylines(image, [pts], isClosed=True, color=color, thickness=thickness)

        cx, cy = region["center_uv"]
        tip_len = min(region["size_uv"]) * 0.35
        dx = np.cos(region["theta_rad"]) * tip_len
        dy = np.sin(region["theta_rad"]) * tip_len

        p0 = (int(round(cx)), int(round(cy)))
        p1 = (int(round(cx + dx)), int(round(cy + dy)))
        cv2.arrowedLine(image, p0, p1, color, 2, tipLength=0.25)

        text_pt = tuple(pts[0])
        cv2.putText(
            image, label, (int(text_pt[0]), max(20, int(text_pt[1]) - 5)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2
        )

    # --------------------------------------------------
    # auto ROI preview
    # --------------------------------------------------
    def preview_regions(self, wall_region, tile_regions, min_mm, max_mm):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

            if self.localizer.depth_image is None:
                continue

            vis = self.depth_to_colormap(self.localizer.depth_image, min_mm, max_mm)

            self.draw_region(vis, wall_region, (0, 255, 255), thickness=2, label="WALL")
            for item in tile_regions:
                self.draw_region(vis, item, (0, 255, 0), thickness=2, label=item["name"])

            help1 = "AUTO ROI from /yolo/tile_array | c/ENTER: continue | q/ESC: cancel"
            help2 = "wall: depth ROI fixed | tile: rgb->depth scaled + rotated"
            cv2.putText(vis, help1, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 2)
            cv2.putText(vis, help2, (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 2)

            cv2.imshow(self.window_name, vis)
            key = cv2.waitKey(30) & 0xFF
            if key in [ord('c'), 13]:
                return True
            if key in [ord('q'), 27]:
                return False

        return False

    # --------------------------------------------------
    # visualization helpers
    # --------------------------------------------------
    def depth_to_colormap(self, depth_image, min_mm, max_mm):
        depth = depth_image.astype(np.float32)
        vis = np.zeros_like(depth, dtype=np.uint8)

        valid = (depth >= min_mm) & (depth <= max_mm)
        if np.any(valid):
            clipped = np.clip(depth, min_mm, max_mm)
            denom = max(1.0, (max_mm - min_mm))
            norm = ((clipped - min_mm) / denom * 255.0).astype(np.uint8)
            vis[valid] = norm[valid]

        color = cv2.applyColorMap(vis, cv2.COLORMAP_JET)
        color[~valid] = (0, 0, 0)
        return color

    def residual_to_colormap(self, residual, clim_mm):
        img = np.zeros(residual.shape, dtype=np.uint8)
        valid = np.isfinite(residual)

        if np.any(valid):
            clipped = np.clip(residual, -clim_mm, clim_mm)
            norm = ((clipped + clim_mm) / (2.0 * clim_mm) * 255.0).astype(np.uint8)
            img[valid] = norm[valid]

        color = cv2.applyColorMap(img, cv2.COLORMAP_JET)
        color[~valid] = (0, 0, 0)
        return color

    # --------------------------------------------------
    # orientation helpers
    # --------------------------------------------------
    def plane_rotation_from_normal(self, normal):
        z_axis = np.asarray(normal, dtype=np.float64)
        z_axis = z_axis / np.linalg.norm(z_axis)

        ref_x = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        x_axis = ref_x - np.dot(ref_x, z_axis) * z_axis
        x_norm = np.linalg.norm(x_axis)

        if x_norm < 1e-8:
            ref_x = np.array([0.0, 1.0, 0.0], dtype=np.float64)
            x_axis = ref_x - np.dot(ref_x, z_axis) * z_axis
            x_norm = np.linalg.norm(x_axis)

        x_axis = x_axis / x_norm
        y_axis = np.cross(z_axis, x_axis)
        y_axis = y_axis / np.linalg.norm(y_axis)

        x_axis = np.cross(y_axis, z_axis)
        x_axis = x_axis / np.linalg.norm(x_axis)

        R = np.column_stack([x_axis, y_axis, z_axis])
        return R

    def rotation_matrix_to_rpy_deg(self, R):
        R = np.asarray(R, dtype=np.float64)

        sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
        singular = sy < 1e-8

        if not singular:
            roll = np.arctan2(R[2, 1], R[2, 2])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw = np.arctan2(R[1, 0], R[0, 0])
        else:
            roll = np.arctan2(-R[1, 2], R[1, 1])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw = 0.0

        return float(np.degrees(roll)), float(np.degrees(pitch)), float(np.degrees(yaw))

    def plane_normal_to_rpy_deg(self, normal):
        R = self.plane_rotation_from_normal(normal)
        roll, pitch, yaw = self.rotation_matrix_to_rpy_deg(R)
        return roll, pitch, yaw, R

    # --------------------------------------------------
    # 3D helpers
    # --------------------------------------------------
    def pixel_to_camera_point_filtered(
        self,
        u,
        v,
        kernel_size=5,
        min_mm=200,
        max_mm=5000,
        inlier_thresh_mm=80.0,
    ):
        depth_mm = self.localizer.get_filtered_depth_at(
            u=int(u),
            v=int(v),
            kernel_size=kernel_size,
            min_mm=min_mm,
            max_mm=max_mm,
            inlier_thresh_mm=inlier_thresh_mm,
        )
        if depth_mm is None:
            return None, None

        p = self.localizer.pixel_to_camera_3d(int(u), int(v), depth_mm)
        return p, float(depth_mm)

    def fit_plane_from_points_svd(self, points_xyz):
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

    def signed_distance_to_plane(self, point_xyz, normal, d):
        point_xyz = np.asarray(point_xyz, dtype=np.float64)
        return float(np.dot(normal, point_xyz) + d)

    def angle_between_normals_deg(self, n1, n2):
        dot = float(np.clip(np.dot(n1, n2), -1.0, 1.0))
        return float(np.degrees(np.arccos(dot)))

    # --------------------------------------------------
    # region sampling / analysis
    # --------------------------------------------------
    def sample_n_points_in_region(self, region, n_points=25, margin=10):
        if region["type"] == "axis_aligned":
            u1, v1, u2, v2 = region["roi"]

            w = u2 - u1
            h = v2 - v1

            if w <= 2 * margin:
                margin = 0
            if h <= 2 * margin:
                margin = 0

            uu1 = u1 + margin
            uu2 = u2 - margin
            vv1 = v1 + margin
            vv2 = v2 - margin

            if uu2 <= uu1 or vv2 <= vv1:
                uu1, uu2 = u1, u2
                vv1, vv2 = v1, v2

            grid_n = int(np.ceil(np.sqrt(max(4, n_points))))
            us = np.linspace(uu1, uu2 - 1, grid_n).astype(int)
            vs = np.linspace(vv1, vv2 - 1, grid_n).astype(int)

            pts = [(int(u), int(v)) for v in vs for u in us]
            if len(pts) > n_points:
                idx = np.linspace(0, len(pts) - 1, n_points).astype(int)
                pts = [pts[i] for i in idx]
            return pts

        # rotated rect
        cx, cy = region["center_uv"]
        rw, rh = region["size_uv"]
        theta = region["theta_rad"]

        inner_w = max(2.0, rw - 2.0 * margin)
        inner_h = max(2.0, rh - 2.0 * margin)

        grid_n = int(np.ceil(np.sqrt(max(4, n_points))))
        xs = np.linspace(-0.5 * inner_w, 0.5 * inner_w, grid_n)
        ys = np.linspace(-0.5 * inner_h, 0.5 * inner_h, grid_n)

        c = np.cos(theta)
        s = np.sin(theta)

        pts = []
        for yy in ys:
            for xx in xs:
                u = cx + xx * c - yy * s
                v = cy + xx * s + yy * c
                u, v = self.clamp_point(u, v)
                pts.append((int(round(u)), int(round(v))))

        dedup = []
        seen = set()
        for p in pts:
            if p not in seen and self.point_in_region(p[0], p[1], region):
                seen.add(p)
                dedup.append(p)

        if len(dedup) > n_points:
            idx = np.linspace(0, len(dedup) - 1, n_points).astype(int)
            dedup = [dedup[i] for i in idx]

        return dedup

    def get_plane_sample_points_camera(
        self,
        region,
        n_points,
        margin,
        kernel_size,
        min_mm,
        max_mm,
        inlier_thresh_mm,
    ):
        uv_samples = self.sample_n_points_in_region(
            region=region,
            n_points=n_points,
            margin=margin,
        )

        points_xyz = []
        valid_uv = []
        valid_depths = []

        for (u, v) in uv_samples:
            p, depth_mm = self.pixel_to_camera_point_filtered(
                u=u,
                v=v,
                kernel_size=kernel_size,
                min_mm=min_mm,
                max_mm=max_mm,
                inlier_thresh_mm=inlier_thresh_mm,
            )
            if p is None:
                continue

            points_xyz.append(p)
            valid_uv.append((u, v))
            valid_depths.append(depth_mm)

        if len(points_xyz) < 3:
            return None, None, None

        return np.asarray(points_xyz, dtype=np.float64), valid_uv, valid_depths

    def compute_region_plane_distance_map(
        self,
        region,
        normal,
        d,
        stride=2,
        kernel_size=5,
        min_mm=200,
        max_mm=5000,
        inlier_thresh_mm=80.0,
    ):
        u1, v1, u2, v2 = region["roi"]

        h = v2 - v1
        w = u2 - u1

        residual = np.full((h, w), np.nan, dtype=np.float64)
        valid_mask = np.zeros((h, w), dtype=bool)
        region_depth = np.full((h, w), np.nan, dtype=np.float64)

        step = max(1, int(stride))
        for vv in range(v1, v2, step):
            for uu in range(u1, u2, step):
                if not self.point_in_region(uu, vv, region):
                    continue

                p, depth_mm = self.pixel_to_camera_point_filtered(
                    u=uu,
                    v=vv,
                    kernel_size=kernel_size,
                    min_mm=min_mm,
                    max_mm=max_mm,
                    inlier_thresh_mm=inlier_thresh_mm,
                )
                if p is None:
                    continue

                dist = self.signed_distance_to_plane(p, normal, d)
                rr = vv - v1
                cc = uu - u1
                residual[rr, cc] = dist
                region_depth[rr, cc] = depth_mm
                valid_mask[rr, cc] = True

        return region_depth, residual, valid_mask

    def get_center_point_camera(
        self,
        region,
        kernel_size,
        min_mm,
        max_mm,
        inlier_thresh_mm,
    ):
        uc = int(round(region["center_uv"][0]))
        vc = int(round(region["center_uv"][1]))
        p, depth_mm = self.pixel_to_camera_point_filtered(
            u=uc,
            v=vc,
            kernel_size=kernel_size,
            min_mm=min_mm,
            max_mm=max_mm,
            inlier_thresh_mm=inlier_thresh_mm,
        )
        return (uc, vc), p, depth_mm

    def analyze_region_3d(
        self,
        region,
        n_points,
        margin,
        kernel_size,
        min_mm,
        max_mm,
        inlier_thresh_mm,
        residual_stride,
    ):
        points_xyz, sample_uv, sample_depths = self.get_plane_sample_points_camera(
            region=region,
            n_points=n_points,
            margin=margin,
            kernel_size=kernel_size,
            min_mm=min_mm,
            max_mm=max_mm,
            inlier_thresh_mm=inlier_thresh_mm,
        )
        if points_xyz is None:
            return None

        plane = self.fit_plane_from_points_svd(points_xyz)
        if plane is None:
            return None

        normal, d, centroid = plane
        roll_deg, pitch_deg, yaw_deg, R_plane = self.plane_normal_to_rpy_deg(normal)

        region_depth, residual, valid_mask = self.compute_region_plane_distance_map(
            region=region,
            normal=normal,
            d=d,
            stride=residual_stride,
            kernel_size=kernel_size,
            min_mm=min_mm,
            max_mm=max_mm,
            inlier_thresh_mm=inlier_thresh_mm,
        )

        vals = residual[valid_mask]
        if vals.size > 0:
            rmse = float(np.sqrt(np.mean(vals ** 2)))
            res_mean = float(np.mean(vals))
            res_min = float(np.min(vals))
            res_max = float(np.max(vals))
        else:
            rmse = float("nan")
            res_mean = float("nan")
            res_min = float("nan")
            res_max = float("nan")

        center_uv, center_point, center_depth = self.get_center_point_camera(
            region=region,
            kernel_size=kernel_size,
            min_mm=min_mm,
            max_mm=max_mm,
            inlier_thresh_mm=inlier_thresh_mm,
        )

        if center_point is not None:
            center_plane_dist = self.signed_distance_to_plane(center_point, normal, d)
        else:
            center_plane_dist = float("nan")

        return {
            "name": region["name"],
            "region": region,
            "roi": region["roi"],
            "sample_points_xyz": points_xyz,
            "sample_uv": sample_uv,
            "sample_depths": sample_depths,
            "plane_normal": normal,
            "plane_d": d,
            "plane_centroid": centroid,
            "plane_rotation": R_plane,
            "roll_deg": roll_deg,
            "pitch_deg": pitch_deg,
            "yaw_deg": yaw_deg,
            "roi_depth": region_depth,
            "residual": residual,
            "valid_mask": valid_mask,
            "valid_count": int(np.count_nonzero(valid_mask)),
            "rmse": rmse,
            "res_mean": res_mean,
            "res_min": res_min,
            "res_max": res_max,
            "center_uv": center_uv,
            "center_point": center_point,
            "center_depth": center_depth,
            "center_plane_dist": center_plane_dist,
        }

    # --------------------------------------------------
    # web helpers
    # --------------------------------------------------
    def flatten_patch_to_vertices_indices(self, X, Y, Z):
        h, w = X.shape
        vertices = []
        for r in range(h):
            for c in range(w):
                vertices.append([
                    float(X[r, c]) / 1000.0,
                    float(Y[r, c]) / 1000.0,
                    float(Z[r, c]) / 1000.0,
                ])

        indices = []
        for r in range(h - 1):
            for c in range(w - 1):
                i00 = r * w + c
                i01 = r * w + (c + 1)
                i10 = (r + 1) * w + c
                i11 = (r + 1) * w + (c + 1)

                indices.extend([i00, i10, i11])
                indices.extend([i00, i11, i01])

        return vertices, indices

    def build_plane_axes_from_normal(self, normal):
        z_axis = np.asarray(normal, dtype=np.float64)
        z_axis = z_axis / np.linalg.norm(z_axis)

        ref_x = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        x_axis = ref_x - np.dot(ref_x, z_axis) * z_axis

        if np.linalg.norm(x_axis) < 1e-8:
            ref_x = np.array([0.0, 1.0, 0.0], dtype=np.float64)
            x_axis = ref_x - np.dot(ref_x, z_axis) * z_axis

        x_axis = x_axis / np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        y_axis = y_axis / np.linalg.norm(y_axis)

        x_axis = np.cross(y_axis, z_axis)
        x_axis = x_axis / np.linalg.norm(x_axis)

        return x_axis, y_axis, z_axis

    def build_roi_plane_patch_from_points(self, points_xyz, centroid, normal, resolution=10, margin_mm=0.0):
        pts = np.asarray(points_xyz, dtype=np.float64)
        centroid = np.asarray(centroid, dtype=np.float64)

        x_axis, y_axis, _ = self.build_plane_axes_from_normal(normal)

        rel = pts - centroid[None, :]
        xs = rel @ x_axis
        ys = rel @ y_axis

        xmin = float(np.min(xs)) - margin_mm
        xmax = float(np.max(xs)) + margin_mm
        ymin = float(np.min(ys)) - margin_mm
        ymax = float(np.max(ys)) + margin_mm

        uu = np.linspace(xmin, xmax, resolution)
        vv = np.linspace(ymin, ymax, resolution)
        U, V = np.meshgrid(uu, vv)

        patch = (
            centroid[None, None, :]
            + U[..., None] * x_axis[None, None, :]
            + V[..., None] * y_axis[None, None, :]
        )

        X = patch[..., 0]
        Y = patch[..., 1]
        Z = patch[..., 2]
        return X, Y, Z

    def analysis_to_web_plane(self, analysis, color_rgb):
        X, Y, Z = self.build_roi_plane_patch_from_points(
            points_xyz=analysis["sample_points_xyz"],
            centroid=analysis["plane_centroid"],
            normal=analysis["plane_normal"],
            resolution=12,
            margin_mm=0.0,
        )

        patch_vertices, patch_indices = self.flatten_patch_to_vertices_indices(X, Y, Z)
        sample_points = [self.mm_to_m_list(p) for p in analysis["sample_points_xyz"]]
        centroid = self.mm_to_m_list(analysis["plane_centroid"])

        center_point = None
        if analysis["center_point"] is not None:
            center_point = self.mm_to_m_list(analysis["center_point"])

        region = analysis["region"]
        region_box = self.get_region_box_points(region)

        return {
            "name": analysis["name"],
            "roi": [int(v) for v in analysis["roi"]],
            "region_type": region["type"],
            "region_box_uv": region_box.astype(float).tolist(),
            "sample_points": sample_points,
            "centroid": centroid,
            "normal": [
                float(analysis["plane_normal"][0]),
                float(analysis["plane_normal"][1]),
                float(analysis["plane_normal"][2]),
            ],
            "center_point": center_point,
            "patch_vertices": patch_vertices,
            "patch_indices": patch_indices,
            "rmse_mm": float(analysis["rmse"]),
            "res_mean_mm": float(analysis["res_mean"]),
            "res_min_mm": float(analysis["res_min"]),
            "res_max_mm": float(analysis["res_max"]),
            "roll_deg": float(analysis["roll_deg"]),
            "pitch_deg": float(analysis["pitch_deg"]),
            "yaw_deg": float(analysis["yaw_deg"]),
            "color": [float(color_rgb[0]), float(color_rgb[1]), float(color_rgb[2])],
        }

    def publish_web_viz_direct(self, wall_analysis, tile_analyses):
        payload = {
            "frame_id": "camera_color_optical_frame",
            "timestamp_sec": float(time.time()),
            "wall": self.analysis_to_web_plane(wall_analysis, color_rgb=[1.0, 0.9, 0.1]),
            "tiles": [self.analysis_to_web_plane(t, color_rgb=[0.1, 1.0, 0.2]) for t in tile_analyses],
        }

        payload_str = json.dumps(payload)

        msg = String()
        msg.data = payload_str
        # self.web_viz_pub.publish(msg)

    # --------------------------------------------------
    # result canvas
    # --------------------------------------------------
    def set_axes_equal_by_wall(self, ax, wall_analysis, tile_analyses):
        all_pts = [wall_analysis["sample_points_xyz"]]
        for t in tile_analyses:
            all_pts.append(t["sample_points_xyz"])

        pts = np.vstack(all_pts)

        xmin, ymin, zmin = np.min(pts, axis=0)
        xmax, ymax, zmax = np.max(pts, axis=0)

        cx = 0.5 * (xmin + xmax)
        cy = 0.5 * (ymin + ymax)
        cz = 0.5 * (zmin + zmax)

        rx = 0.5 * (xmax - xmin)
        ry = 0.5 * (ymax - ymin)
        rz = 0.5 * (zmax - zmin)

        r = max(rx, ry, rz, 1.0)

        ax.set_xlim(cx - r, cx + r)
        ax.set_ylim(cy - r, cy + r)
        ax.set_zlim(cz - r, cz + r)
        ax.set_box_aspect([1, 1, 1])

    def compose_result_canvas(
        self,
        wall_analysis,
        tile_analyses,
        min_mm,
        max_mm,
        residual_clim_mm,
        comparison_lines,
    ):
        full = self.depth_to_colormap(self.localizer.depth_image, min_mm, max_mm)

        self.draw_region(full, wall_analysis["region"], (0, 255, 255), thickness=2, label="WALL")
        for (u, v) in wall_analysis["sample_uv"]:
            cv2.circle(full, (u, v), 2, (0, 255, 255), -1)

        for t in tile_analyses:
            self.draw_region(full, t["region"], (0, 255, 0), thickness=2, label=t["name"])
            for (u, v) in t["sample_uv"]:
                cv2.circle(full, (u, v), 2, (0, 255, 0), -1)

        wall_resid = self.residual_to_colormap(wall_analysis["residual"], residual_clim_mm)
        wall_resid[~wall_analysis["valid_mask"]] = (0, 0, 0)

        panels = []
        for t in tile_analyses[:3]:
            img = self.residual_to_colormap(t["residual"], residual_clim_mm)
            img[~t["valid_mask"]] = (0, 0, 0)
            panels.append((t["name"], img))

        panel_h = 300
        panel_w = 400

        full_panel = cv2.resize(full, (panel_w, panel_h))
        cv2.putText(full_panel, "Full Depth + Regions + Sample Points", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.68, (255, 255, 255), 2)

        wall_panel = cv2.resize(wall_resid, (panel_w, panel_h))
        cv2.putText(wall_panel, "Wall Plane Distance Map", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.72, (255, 255, 255), 2)

        second_row = [full_panel, wall_panel]

        for name, img in panels:
            p = cv2.resize(img, (panel_w, panel_h))
            cv2.putText(p, f"{name} Plane Distance Map", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
            second_row.append(p)

        while len(second_row) < 4:
            second_row.append(np.zeros((panel_h, panel_w, 3), dtype=np.uint8))

        top = np.hstack(second_row[:2])
        bottom = np.hstack(second_row[2:4])
        canvas = np.vstack([top, bottom])

        info_h = max(260, 28 * (len(comparison_lines) + 2))
        info_panel = np.zeros((info_h, canvas.shape[1], 3), dtype=np.uint8)

        y = 30
        for line in comparison_lines:
            cv2.putText(info_panel, line, (15, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 255, 255), 2)
            y += 26

        final = np.vstack([canvas, info_panel])
        return final

    def plot_roi_planes_3d_matplotlib(self, wall_analysis, tile_analyses):
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection="3d")

        w_pts = wall_analysis["sample_points_xyz"]
        w_c = wall_analysis["plane_centroid"]
        w_n = wall_analysis["plane_normal"]

        ax.scatter(w_pts[:, 0], w_pts[:, 1], w_pts[:, 2], s=25, label="wall points")

        X, Y, Z = self.build_roi_plane_patch_from_points(
            points_xyz=w_pts,
            centroid=w_c,
            normal=w_n,
            resolution=12,
            margin_mm=0.0,
        )
        ax.plot_surface(X, Y, Z, alpha=0.35)

        ax.quiver(w_c[0], w_c[1], w_c[2], w_n[0], w_n[1], w_n[2], length=40.0, normalize=True)
        ax.text(w_c[0], w_c[1], w_c[2], "WALL")

        for t in tile_analyses:
            pts = t["sample_points_xyz"]
            c = t["plane_centroid"]
            n = t["plane_normal"]

            ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], s=20)

            X, Y, Z = self.build_roi_plane_patch_from_points(
                points_xyz=pts,
                centroid=c,
                normal=n,
                resolution=10,
                margin_mm=0.0,
            )
            ax.plot_surface(X, Y, Z, alpha=0.35)

            ax.quiver(c[0], c[1], c[2], n[0], n[1], n[2], length=25.0, normalize=True)
            ax.text(c[0], c[1], c[2], t["name"])

        self.set_axes_equal_by_wall(ax, wall_analysis, tile_analyses)

        ax.set_xlabel("X (mm)")
        ax.set_ylabel("Y (mm)")
        ax.set_zlabel("Z (mm)")
        ax.set_title("ROI Plane Visualization (Wall: depth ROI, Tile: rotated RGB->Depth ROI)")
        plt.tight_layout()
        plt.show()

    # --------------------------------------------------
    # main
    # --------------------------------------------------
    def run_test(self):
        wait_timeout_sec = float(self.get_parameter("wait_timeout_sec").value)
        min_mm = int(self.get_parameter("min_mm").value)
        max_mm = int(self.get_parameter("max_mm").value)
        n_points = int(self.get_parameter("sample_n_points").value)
        margin = int(self.get_parameter("sample_margin_px").value)
        kernel_size = int(self.get_parameter("depth_kernel_size").value)
        inlier_thresh_mm = float(self.get_parameter("depth_inlier_thresh_mm").value)
        residual_clim_mm = float(self.get_parameter("residual_clim_mm").value)
        residual_stride = int(self.get_parameter("residual_stride").value)

        if not self.wait_until_ready(timeout_sec=wait_timeout_sec):
            self.get_logger().error("[WallTileInspection3DNode] failed: sensor data not ready")
            return

        wall_region = self.create_wall_region()
        self.get_logger().info(f"[WallTileInspection3DNode] wall region ready: {wall_region['roi']}")

        if not self.wait_for_tile_array(timeout_sec=5.0):
            self.get_logger().error("[WallTileInspection3DNode] failed: /yolo/tile_array not ready")
            return

        self.tile_regions = self.build_tile_regions_from_yolo(margin_px=self.tile_margin_px_depth)
        if len(self.tile_regions) == 0:
            self.get_logger().error("[WallTileInspection3DNode] no tile regions generated from YOLO")
            return

        self.get_logger().info(
            f"[WallTileInspection3DNode] auto-generated {len(self.tile_regions)} rotated tile regions"
        )
        for item in self.tile_regions:
            self.get_logger().info(
                f"[AUTO TILE] {item['name']} "
                f"rgb_center={item['rgb_center']} rgb_size={item['rgb_size']} "
                f"depth_center=({item['center_uv'][0]:.2f},{item['center_uv'][1]:.2f}) "
                f"depth_size=({item['size_uv'][0]:.2f},{item['size_uv'][1]:.2f}) "
                f"theta={item['theta_deg']:.2f}deg bbox={item['roi']} conf={item['conf_score']:.3f}"
            )

        ok = self.preview_regions(
            wall_region=wall_region,
            tile_regions=self.tile_regions,
            min_mm=min_mm,
            max_mm=max_mm,
        )
        if not ok:
            self.get_logger().warn("[WallTileInspection3DNode] preview cancelled")
            return

        wall_analysis = self.analyze_region_3d(
            region=wall_region,
            n_points=n_points,
            margin=margin,
            kernel_size=kernel_size,
            min_mm=min_mm,
            max_mm=max_mm,
            inlier_thresh_mm=inlier_thresh_mm,
            residual_stride=residual_stride,
        )
        if wall_analysis is None:
            self.get_logger().error("[WallTileInspection3DNode] failed to analyze wall region")
            return

        tile_analyses = []
        for region in self.tile_regions:
            a = self.analyze_region_3d(
                region=region,
                n_points=n_points,
                margin=margin,
                kernel_size=kernel_size,
                min_mm=min_mm,
                max_mm=max_mm,
                inlier_thresh_mm=inlier_thresh_mm,
                residual_stride=residual_stride,
            )
            if a is None:
                self.get_logger().warn(f"[WallTileInspection3DNode] failed to analyze tile region: {region['name']}")
                continue
            tile_analyses.append(a)

        if len(tile_analyses) == 0:
            self.get_logger().error("[WallTileInspection3DNode] no valid tile analyses")
            return

        comparison_lines = []

        wn = wall_analysis["plane_normal"]
        wd = wall_analysis["plane_d"]

        wr = wall_analysis["roll_deg"]
        wp = wall_analysis["pitch_deg"]
        wy = wall_analysis["yaw_deg"]

        comparison_lines.append("[Wall 3D Plane]")
        comparison_lines.append(
            f"normal=[{wn[0]:.4f}, {wn[1]:.4f}, {wn[2]:.4f}], "
            f"rpy=({wr:.2f},{wp:.2f},{wy:.2f})deg, "
            f"d={wd:.3f}, rmse={wall_analysis['rmse']:.3f}mm, "
            f"valid={wall_analysis['valid_count']}"
        )

        comparison_lines.append("[Wall vs Tiles]")
        for t in tile_analyses:
            tn = t["plane_normal"]
            td = t["plane_d"]

            angle_deg = self.angle_between_normals_deg(wn, tn)

            center_offset_wall = float("nan")
            mean_offset_wall = float("nan")

            if t["center_point"] is not None:
                center_offset_wall = self.signed_distance_to_plane(t["center_point"], wn, wd)

            sample_offsets_to_wall = [self.signed_distance_to_plane(p, wn, wd) for p in t["sample_points_xyz"]]
            if len(sample_offsets_to_wall) > 0:
                mean_offset_wall = float(np.mean(sample_offsets_to_wall))

            dr = t["roll_deg"] - wr
            dp = t["pitch_deg"] - wp
            dy = t["yaw_deg"] - wy

            line = (
                f"{t['name']}: "
                f"rpy=({t['roll_deg']:.2f},{t['pitch_deg']:.2f},{t['yaw_deg']:.2f})deg, "
                f"drpy=({dr:.2f},{dp:.2f},{dy:.2f})deg, "
                f"tilt={angle_deg:.3f}deg, "
                f"center_offset_to_wall={center_offset_wall:.3f}mm, "
                f"mean_offset_to_wall={mean_offset_wall:.3f}mm, "
                f"tile_rmse={t['rmse']:.3f}mm"
            )
            comparison_lines.append(line)

            self.get_logger().info(line)
            self.get_logger().info(
                f"[{t['name']}] normal=[{tn[0]:.4f}, {tn[1]:.4f}, {tn[2]:.4f}], "
                f"d={td:.3f}, valid={t['valid_count']}"
            )

        comparison_lines.append("[Tile vs Tile]")
        for i in range(len(tile_analyses)):
            for j in range(i + 1, len(tile_analyses)):
                t1 = tile_analyses[i]
                t2 = tile_analyses[j]

                tilt_diff = self.angle_between_normals_deg(t1["plane_normal"], t2["plane_normal"])

                offset1 = float("nan")
                offset2 = float("nan")

                if t1["center_point"] is not None:
                    offset1 = self.signed_distance_to_plane(t1["center_point"], wn, wd)
                if t2["center_point"] is not None:
                    offset2 = self.signed_distance_to_plane(t2["center_point"], wn, wd)

                offset_diff_wall = float("nan")
                if np.isfinite(offset1) and np.isfinite(offset2):
                    offset_diff_wall = offset2 - offset1

                center_depth_diff = float("nan")
                if t1["center_point"] is not None and t2["center_point"] is not None:
                    center_depth_diff = float(t2["center_point"][2] - t1["center_point"][2])

                dr = t2["roll_deg"] - t1["roll_deg"]
                dp = t2["pitch_deg"] - t1["pitch_deg"]
                dy = t2["yaw_deg"] - t1["yaw_deg"]

                line = (
                    f"{t1['name']} vs {t2['name']}: "
                    f"drpy=({dr:.2f},{dp:.2f},{dy:.2f})deg, "
                    f"tilt_diff={tilt_diff:.3f}deg, "
                    f"wall_offset_diff={offset_diff_wall:.3f}mm, "
                    f"center_depth_diff={center_depth_diff:.3f}mm"
                )
                comparison_lines.append(line)

        result = self.compose_result_canvas(
            wall_analysis=wall_analysis,
            tile_analyses=tile_analyses,
            min_mm=min_mm,
            max_mm=max_mm,
            residual_clim_mm=residual_clim_mm,
            comparison_lines=comparison_lines,
        )

        self.publish_web_viz_direct(wall_analysis, tile_analyses)
        self.plot_roi_planes_3d_matplotlib(wall_analysis, tile_analyses)

        cv2.namedWindow(self.result_window_name, cv2.WINDOW_NORMAL)
        while rclpy.ok():
            cv2.imshow(self.result_window_name, result)
            key = cv2.waitKey(30) & 0xFF

            if key == ord('s'):
                save_path = "/tmp/wall_tile_inspection_3d_result.png"
                cv2.imwrite(save_path, result)
                self.get_logger().info(f"[WallTileInspection3DNode] saved result: {save_path}")
            elif key in [ord('q'), 27]:
                break

        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = WallTileInspection3DNode()

    try:
        node.run_test()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"[WallTileInspection3DNode] exception: {e}")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()