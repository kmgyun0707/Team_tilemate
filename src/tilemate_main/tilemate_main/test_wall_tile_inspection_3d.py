#!/usr/bin/env python3
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from tilemate_main.depth_localizer import DepthLocalizer
import json
from std_msgs.msg import String
import json
import asyncio
import threading
import websockets



class WallTileInspection3DNode(Node):
    def __init__(self):
        super().__init__("wall_tile_inspection_3d_node")

        self.declare_parameter("gripper2cam_path", "")
        self.declare_parameter("use_inverse", True)

        self.declare_parameter("min_mm", 200)
        self.declare_parameter("max_mm", 3000)
        self.declare_parameter("wait_timeout_sec", 5.0)

        # n-point plane fitting
        self.declare_parameter("sample_n_points", 25)
        self.declare_parameter("sample_margin_px", 10)
        self.declare_parameter("depth_kernel_size", 5)
        self.declare_parameter("depth_inlier_thresh_mm", 80.0)

        # residual map
        self.declare_parameter("residual_clim_mm", 15.0)
        self.declare_parameter("residual_stride", 2)

        self.window_name = "Wall/Tile ROI Selector (3D Plane)"
        self.dragging = False
        self.pt1 = None
        self.pt2 = None

        self.mode = "tile"   # wall -> tile
        self.wall_roi = (248, 87, 597, 360)
        self.tile_rois = []

        gripper2cam_path = self.get_parameter("gripper2cam_path").get_parameter_value().string_value
        use_inverse = self.get_parameter("use_inverse").get_parameter_value().bool_value

        self.localizer = DepthLocalizer(
            node=self,
            gripper2cam_path=gripper2cam_path,
            use_inverse=use_inverse
        )
        self._ws_clients = set()
        self._ws_loop = None
        self._ws_thread = None
        self._start_websocket_server()
        self.web_viz_pub = self.create_publisher(String, "/wall_tile_inspection/web_viz", 10)
        self.get_logger().info("[WallTileInspection3DNode] initialized")

    # --------------------------------------------------
    # ready
    # --------------------------------------------------

    def _start_websocket_server(self, host="0.0.0.0", port=8765):
        def run_loop():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            self._ws_loop = loop

            async def handler(websocket):
                self.get_logger().info("[WebSocket] client connected")
                self._ws_clients.add(websocket)
                try:
                    await websocket.wait_closed()
                finally:
                    self._ws_clients.discard(websocket)
                    self.get_logger().info("[WebSocket] client disconnected")

            async def start_server():
                server = await websockets.serve(handler, host, port)
                self.get_logger().info(f"[WebSocket] serving on ws://{host}:{port}")
                await server.wait_closed()

            loop.run_until_complete(start_server())

        self._ws_thread = threading.Thread(target=run_loop, daemon=True)
        self._ws_thread.start()

    async def _broadcast_ws(self, payload_str):
        dead = []
        for ws in list(self._ws_clients):
            try:
                await ws.send(payload_str)
            except Exception:
                dead.append(ws)

        for ws in dead:
            self._ws_clients.discard(ws)

    def publish_web_viz_direct(self, wall_analysis, tile_analyses):
        payload = {
            "frame_id": "camera_color_optical_frame",
            "timestamp_sec": float(time.time()),
            "wall": self.analysis_to_web_plane(
                wall_analysis,
                color_rgb=[1.0, 0.9, 0.1],
            ),
            "tiles": [
                self.analysis_to_web_plane(t, color_rgb=[0.1, 1.0, 0.2])
                for t in tile_analyses
            ],
        }

        payload_str = json.dumps(payload)

        if self._ws_loop is not None:
            asyncio.run_coroutine_threadsafe(
                self._broadcast_ws(payload_str),
                self._ws_loop
            )
            self.get_logger().info("[WallTileInspection3DNode] broadcasted result to websocket clients")

    def mm_to_m_list(self, p):
        return [float(p[0]) / 1000.0, float(p[1]) / 1000.0, float(p[2]) / 1000.0]

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

        return {
            "name": analysis["name"],
            "roi": [int(v) for v in analysis["roi"]],
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
            "roll_deg": float(analysis["roll_deg"]),
            "pitch_deg": float(analysis["pitch_deg"]),
            "yaw_deg": float(analysis["yaw_deg"]),
            "color": [float(color_rgb[0]), float(color_rgb[1]), float(color_rgb[2])],
        }
  
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

    # --------------------------------------------------
    # ROI helpers
    # --------------------------------------------------
    def mm_to_m_list(self, p):
        return [float(p[0]) / 1000.0, float(p[1]) / 1000.0, float(p[2]) / 1000.0]

    def flatten_patch_to_vertices_indices(self, X, Y, Z):
        """
        meshgrid 형태의 X,Y,Z를 Three.js용 vertices / indices로 변환
        return:
            vertices: [[x,y,z], ...]  (meter)
            indices:  [i0,i1,i2, ...]
        """
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

                # tri 1
                indices.extend([i00, i10, i11])
                # tri 2
                indices.extend([i00, i11, i01])

        return vertices, indices

    def analysis_to_web_plane(self, analysis, color_rgb):
        """
        하나의 분석 결과를 웹용 dict로 변환
        """
        X, Y, Z = self.build_roi_plane_patch_from_points(
            points_xyz=analysis["sample_points_xyz"],
            centroid=analysis["plane_centroid"],
            normal=analysis["plane_normal"],
            resolution=12,
            margin_mm=0.0,
        )

        patch_vertices, patch_indices = self.flatten_patch_to_vertices_indices(X, Y, Z)

        sample_points = [
            self.mm_to_m_list(p) for p in analysis["sample_points_xyz"]
        ]

        centroid = self.mm_to_m_list(analysis["plane_centroid"])
        normal = [
            float(analysis["plane_normal"][0]),
            float(analysis["plane_normal"][1]),
            float(analysis["plane_normal"][2]),
        ]

        center_point = None
        if analysis["center_point"] is not None:
            center_point = self.mm_to_m_list(analysis["center_point"])

        return {
            "name": analysis["name"],
            "roi": [int(v) for v in analysis["roi"]],
            "sample_points": sample_points,
            "centroid": centroid,
            "normal": normal,
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

    def publish_web_viz(self, wall_analysis, tile_analyses):
        """
        웹에서 바로 Three.js로 렌더 가능한 JSON publish
        """
        msg = String()

        payload = {
            "frame_id": "camera_color_optical_frame",
            "timestamp_sec": float(time.time()),
            "wall": self.analysis_to_web_plane(
                wall_analysis,
                color_rgb=[1.0, 0.9, 0.1],
            ),
            "tiles": [
                self.analysis_to_web_plane(t, color_rgb=[0.1, 1.0, 0.2])
                for t in tile_analyses
            ],
        }

        msg.data = json.dumps(payload)
        self.web_viz_pub.publish(msg)
        self.get_logger().info("[WallTileInspection3DNode] published /wall_tile_inspection/web_viz")
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
        """
        plane z-axis = normal
        plane x-axis = camera x-axis projected onto plane
        plane y-axis = z x x
        return R (3x3), columns=[x_axis, y_axis, z_axis]
        """
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

        # re-orthogonalize
        x_axis = np.cross(y_axis, z_axis)
        x_axis = x_axis / np.linalg.norm(x_axis)

        R = np.column_stack([x_axis, y_axis, z_axis])
        return R

    def rotation_matrix_to_rpy_deg(self, R):
        """
        ZYX convention
        returns roll(x), pitch(y), yaw(z) in degrees
        """
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

        return (
            float(np.degrees(roll)),
            float(np.degrees(pitch)),
            float(np.degrees(yaw)),
        )

    def plane_normal_to_rpy_deg(self, normal):
        R = self.plane_rotation_from_normal(normal)
        roll, pitch, yaw = self.rotation_matrix_to_rpy_deg(R)
        return roll, pitch, yaw, R

    # --------------------------------------------------
    # 3D plane helpers
    # --------------------------------------------------

    def sample_n_points_in_roi(self, roi, n_points=25, margin=10):
        """
        ROI 내부에서 균등 격자 기반 샘플점 생성
        """
        u1, v1, u2, v2 = self.clamp_roi(*roi)

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

        pts = []
        for v in vs:
            for u in us:
                pts.append((int(u), int(v)))

        if len(pts) > n_points:
            idx = np.linspace(0, len(pts) - 1, n_points).astype(int)
            pts = [pts[i] for i in idx]

        return pts

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
            u=u,
            v=v,
            kernel_size=kernel_size,
            min_mm=min_mm,
            max_mm=max_mm,
            inlier_thresh_mm=inlier_thresh_mm,
        )
        if depth_mm is None:
            return None, None

        p = self.localizer.pixel_to_camera_3d(u, v, depth_mm)
        return p, float(depth_mm)

    def fit_plane_from_points_svd(self, points_xyz):
        """
        points_xyz: (N,3)
        return: normal(unit), d, centroid
        plane: n·x + d = 0
        """
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

        # camera +Z 방향 기준 정리
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
    # ROI -> 3D analysis
    # --------------------------------------------------

    def get_plane_sample_points_camera(
        self,
        roi,
        n_points,
        margin,
        kernel_size,
        min_mm,
        max_mm,
        inlier_thresh_mm,
    ):
        uv_samples = self.sample_n_points_in_roi(
            roi=roi,
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

    def compute_roi_plane_distance_map(
        self,
        roi,
        normal,
        d,
        stride=2,
        kernel_size=5,
        min_mm=200,
        max_mm=5000,
        inlier_thresh_mm=80.0,
    ):
        """
        ROI 안의 각 유효 픽셀을 3D로 변환 후 plane까지 signed distance(mm) 계산
        """
        u1, v1, u2, v2 = self.clamp_roi(*roi)

        h = v2 - v1
        w = u2 - u1

        residual = np.full((h, w), np.nan, dtype=np.float64)
        valid_mask = np.zeros((h, w), dtype=bool)
        roi_depth = np.full((h, w), np.nan, dtype=np.float64)

        step = max(1, int(stride))
        for vv in range(v1, v2, step):
            for uu in range(u1, u2, step):
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
                roi_depth[rr, cc] = depth_mm
                valid_mask[rr, cc] = True

        return roi_depth, residual, valid_mask

    def get_center_point_camera(
        self,
        roi,
        kernel_size,
        min_mm,
        max_mm,
        inlier_thresh_mm,
    ):
        uc, vc = self.roi_center_uv(roi)
        p, depth_mm = self.pixel_to_camera_point_filtered(
            u=uc,
            v=vc,
            kernel_size=kernel_size,
            min_mm=min_mm,
            max_mm=max_mm,
            inlier_thresh_mm=inlier_thresh_mm,
        )
        return (uc, vc), p, depth_mm

    def analyze_roi_3d(
        self,
        name,
        roi,
        n_points,
        margin,
        kernel_size,
        min_mm,
        max_mm,
        inlier_thresh_mm,
        residual_stride,
    ):
        points_xyz, sample_uv, sample_depths = self.get_plane_sample_points_camera(
            roi=roi,
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

        roi_depth, residual, valid_mask = self.compute_roi_plane_distance_map(
            roi=roi,
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
            roi=roi,
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
            "name": name,
            "roi": roi,
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
            "roi_depth": roi_depth,
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
    # mouse callback
    # --------------------------------------------------

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.dragging = True
            self.pt1 = (x, y)
            self.pt2 = (x, y)

        elif event == cv2.EVENT_MOUSEMOVE:
            if self.dragging:
                self.pt2 = (x, y)

        elif event == cv2.EVENT_LBUTTONUP:
            self.dragging = False
            self.pt2 = (x, y)

    # --------------------------------------------------
    # ROI selection UI
    # --------------------------------------------------

    def select_rois(self, min_mm, max_mm):
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)

        self.get_logger().info(
            "[WallTileInspection3DNode] "
            "벽 ROI는 고정됨\n"
            "타일 ROI들을 하나씩 선택 후 ENTER\n"
            "콘솔에서 이름 입력\n"
            "등록 끝나면 c 눌러 계산\n"
            "r=tile reset, q=quit"
        )

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

            if self.localizer.depth_image is None:
                blank = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(blank, "Waiting for depth image...", (30, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
                cv2.imshow(self.window_name, blank)
                key = cv2.waitKey(10) & 0xFF
                if key == ord('q'):
                    return False
                continue

            vis = self.depth_to_colormap(self.localizer.depth_image, min_mm, max_mm)

            # 고정 wall ROI 표시
            if self.wall_roi is not None:
                u1, v1, u2, v2 = self.wall_roi
                cv2.rectangle(vis, (u1, v1), (u2, v2), (0, 255, 255), 2)
                cv2.putText(vis, "WALL(FIXED)", (u1, max(20, v1 - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            # 등록된 tile ROI 표시
            for item in self.tile_rois:
                u1, v1, u2, v2 = item["roi"]
                name = item["name"]
                cv2.rectangle(vis, (u1, v1), (u2, v2), (0, 255, 0), 2)
                cv2.putText(vis, name, (u1, max(20, v1 - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)

            # 현재 드래그 중인 ROI 표시
            if self.pt1 is not None and self.pt2 is not None:
                x1, y1 = self.pt1
                x2, y2 = self.pt2
                cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)

            help1 = "MODE: TILE ONLY | ENTER: add tile ROI | c: complete | r: reset tiles | q: quit"
            help2 = "Wall ROI is fixed at (248,87)~(597,360)"
            cv2.putText(vis, help1, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2)
            cv2.putText(vis, help2, (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2)

            cv2.imshow(self.window_name, vis)
            key = cv2.waitKey(10) & 0xFF

            if key == ord('q'):
                return False

            elif key == ord('r'):
                self.tile_rois = []
                self.pt1 = None
                self.pt2 = None
                self.dragging = False
                self.get_logger().info("[WallTileInspection3DNode] reset tile ROIs")

            elif key == ord('c'):
                if self.wall_roi is None:
                    self.get_logger().warn("[WallTileInspection3DNode] wall ROI not set")
                elif len(self.tile_rois) == 0:
                    self.get_logger().warn("[WallTileInspection3DNode] no tile ROI registered")
                else:
                    return True

            elif key in [13, 32]:
                if self.pt1 is None or self.pt2 is None:
                    continue

                x1, y1 = self.pt1
                x2, y2 = self.pt2
                u1, u2 = sorted([x1, x2])
                v1, v2 = sorted([y1, y2])

                try:
                    roi = self.clamp_roi(u1, v1, u2, v2)
                except Exception as e:
                    self.get_logger().warn(f"[WallTileInspection3DNode] invalid tile ROI: {e}")
                    continue

                name = input("타일 이름 입력: ").strip()
                if not name:
                    name = f"tile_{len(self.tile_rois) + 1}"

                self.tile_rois.append({
                    "name": name,
                    "roi": roi,
                })
                self.get_logger().info(f"[WallTileInspection3DNode] tile ROI added: {name}, {roi}")

                self.pt1 = None
                self.pt2 = None
                self.dragging = False

        return False

    # --------------------------------------------------
    # result canvas
    # --------------------------------------------------
    def set_axes_equal_by_wall(self, ax, wall_analysis, tile_analyses):
        """
        wall + tile 전체 점들을 기준으로 동일 스케일 축 설정
        """
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

        wu1, wv1, wu2, wv2 = wall_analysis["roi"]
        cv2.rectangle(full, (wu1, wv1), (wu2, wv2), (0, 255, 255), 2)
        cv2.putText(full, "WALL", (wu1, max(20, wv1 - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        for (u, v) in wall_analysis["sample_uv"]:
            cv2.circle(full, (u, v), 2, (0, 255, 255), -1)

        for t in tile_analyses:
            u1, v1, u2, v2 = t["roi"]
            cv2.rectangle(full, (u1, v1), (u2, v2), (0, 255, 0), 2)
            cv2.putText(full, t["name"], (u1, max(20, v1 - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
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
        cv2.putText(full_panel, "Full Depth + ROIs + Sample Points", (10, 25),
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

    def build_plane_axes_from_normal(self, normal):
        """
        plane의 local axis 생성
        z_axis = normal
        x_axis = camera x축을 plane에 projection
        y_axis = z x x
        """
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

        # re-orthogonalize
        x_axis = np.cross(y_axis, z_axis)
        x_axis = x_axis / np.linalg.norm(x_axis)

        return x_axis, y_axis, z_axis

    def build_roi_plane_patch_from_points(self, points_xyz, centroid, normal, resolution=10, margin_mm=0.0):
        """
        ROI의 실제 3D 샘플점 분포를 기준으로 plane patch 생성
        points_xyz: ROI에서 얻은 3D 점들
        centroid: plane centroid
        normal: plane normal

        return:
            X, Y, Z
        """
        pts = np.asarray(points_xyz, dtype=np.float64)
        centroid = np.asarray(centroid, dtype=np.float64)

        x_axis, y_axis, z_axis = self.build_plane_axes_from_normal(normal)

        # centroid 기준으로 local plane 좌표로 투영
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
    def build_plane_patch(self, centroid, normal, size=80.0, resolution=10):
        """
        centroid, normal 기준으로 평면 patch mesh 생성
        return: X, Y, Z
        """
        normal = np.asarray(normal, dtype=np.float64)
        normal = normal / np.linalg.norm(normal)

        ref = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        if abs(np.dot(ref, normal)) > 0.9:
            ref = np.array([0.0, 1.0, 0.0], dtype=np.float64)

        axis1 = np.cross(normal, ref)
        axis1 = axis1 / np.linalg.norm(axis1)
        axis2 = np.cross(normal, axis1)
        axis2 = axis2 / np.linalg.norm(axis2)

        s = np.linspace(-size, size, resolution)
        uu, vv = np.meshgrid(s, s)

        pts = (
            centroid[None, None, :]
            + uu[..., None] * axis1[None, None, :]
            + vv[..., None] * axis2[None, None, :]
        )

        X = pts[..., 0]
        Y = pts[..., 1]
        Z = pts[..., 2]
        return X, Y, Z
    def plot_roi_planes_3d_matplotlib(self, wall_analysis, tile_analyses):
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection="3d")

        # -------------------------
        # wall
        # -------------------------
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

        ax.quiver(
            w_c[0], w_c[1], w_c[2],
            w_n[0], w_n[1], w_n[2],
            length=40.0, normalize=True
        )
        ax.text(w_c[0], w_c[1], w_c[2], "WALL")

        # -------------------------
        # tiles
        # -------------------------
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

            ax.quiver(
                c[0], c[1], c[2],
                n[0], n[1], n[2],
                length=25.0, normalize=True
            )
            ax.text(c[0], c[1], c[2], t["name"])

        self.set_axes_equal_by_wall(ax, wall_analysis, tile_analyses)

        ax.set_xlabel("X (mm)")
        ax.set_ylabel("Y (mm)")
        ax.set_zlabel("Z (mm)")
        ax.set_title("ROI Plane Visualization (Real ROI Size)")
        plt.tight_layout()
        plt.show()
    # --------------------------------------------------
    # main run
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

        # wall ROI 고정
        self.wall_roi = self.clamp_roi(248, 87, 597, 360)
        self.get_logger().info(f"[WallTileInspection3DNode] fixed wall ROI: {self.wall_roi}")

        ok = self.select_rois(min_mm=min_mm, max_mm=max_mm)
        if not ok:
            self.get_logger().warn("[WallTileInspection3DNode] ROI selection cancelled")
            return

        wall_analysis = self.analyze_roi_3d(
            name="wall",
            roi=self.wall_roi,
            n_points=n_points,
            margin=margin,
            kernel_size=kernel_size,
            min_mm=min_mm,
            max_mm=max_mm,
            inlier_thresh_mm=inlier_thresh_mm,
            residual_stride=residual_stride,
        )
        if wall_analysis is None:
            self.get_logger().error("[WallTileInspection3DNode] failed to analyze wall ROI")
            return

        tile_analyses = []
        for item in self.tile_rois:
            a = self.analyze_roi_3d(
                name=item["name"],
                roi=item["roi"],
                n_points=n_points,
                margin=margin,
                kernel_size=kernel_size,
                min_mm=min_mm,
                max_mm=max_mm,
                inlier_thresh_mm=inlier_thresh_mm,
                residual_stride=residual_stride,
            )
            if a is None:
                self.get_logger().warn(f"[WallTileInspection3DNode] failed to analyze tile ROI: {item['name']}")
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

            sample_offsets_to_wall = []
            for p in t["sample_points_xyz"]:
                sample_offsets_to_wall.append(self.signed_distance_to_plane(p, wn, wd))
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

                tilt_diff = self.angle_between_normals_deg(
                    t1["plane_normal"],
                    t2["plane_normal"]
                )

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

        result_window = "Wall Tile Inspection 3D Result"
        cv2.namedWindow(result_window, cv2.WINDOW_NORMAL)

        while rclpy.ok():
            cv2.imshow(result_window, result)
            key = cv2.waitKey(30) & 0xFF

            if key == ord('s'):
                save_path = "/tmp/wall_tile_inspection_3d_result.png"
                cv2.imwrite(save_path, result)
                self.get_logger().info(f"[WallTileInspection3DNode] saved result: {save_path}")

            elif key == ord('q') or key == 27:
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