#!/usr/bin/env python3
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from custom_tile_msgs.msg import TileArray


class DepthRoiDebugNode(Node):
    def __init__(self):
        super().__init__("depth_roi_debug_node")

        # -----------------------------
        # params
        # -----------------------------
        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("tile_topic", "/yolo/tile_array")

        self.declare_parameter("min_mm", 300)
        self.declare_parameter("max_mm", 400)

        self.declare_parameter("sample_n_points", 25)
        self.declare_parameter("tile_shrink_px_depth", 10)

        self.declare_parameter("debug_window_name", "Depth ROI Debug")
        self.declare_parameter("debug_wait_ms", 1)

        self.declare_parameter("wall_roi_center_u", 422.5)
        self.declare_parameter("wall_roi_center_v", 223.5)
        self.declare_parameter("wall_roi_w", 349.0)
        self.declare_parameter("wall_roi_h", 273.0)
        self.declare_parameter("wall_roi_yaw_deg", 0.00851928332822126)

        # -----------------------------
        # cached params
        # -----------------------------
        self.color_topic = str(self.get_parameter("color_topic").value)
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.tile_topic = str(self.get_parameter("tile_topic").value)

        self.min_mm = int(self.get_parameter("min_mm").value)
        self.max_mm = int(self.get_parameter("max_mm").value)

        self.sample_n_points = int(self.get_parameter("sample_n_points").value)
        self.tile_shrink_px_depth = int(self.get_parameter("tile_shrink_px_depth").value)

        self.debug_window_name = str(self.get_parameter("debug_window_name").value)
        self.debug_wait_ms = int(self.get_parameter("debug_wait_ms").value)

        self.wall_roi_center_u = float(self.get_parameter("wall_roi_center_u").value)
        self.wall_roi_center_v = float(self.get_parameter("wall_roi_center_v").value)
        self.wall_roi_w = float(self.get_parameter("wall_roi_w").value)
        self.wall_roi_h = float(self.get_parameter("wall_roi_h").value)
        self.wall_roi_yaw_deg = float(self.get_parameter("wall_roi_yaw_deg").value)

        # -----------------------------
        # caches
        # -----------------------------
        self.bridge = CvBridge()
        self.color_image = None
        self.depth_image = None
        self.latest_tile_array = None
        self.latest_tile_stamp = None

        # -----------------------------
        # subs
        # -----------------------------
        self.create_subscription(
            Image,
            self.color_topic,
            self.color_callback,
            10,
        )

        self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            10,
        )

        self.create_subscription(
            TileArray,
            self.tile_topic,
            self.tile_array_callback,
            10,
        )

        # timer
        self.timer = self.create_timer(0.1, self.on_timer)

        self.get_logger().info(f"color_topic: {self.color_topic}")
        self.get_logger().info(f"depth_topic: {self.depth_topic}")
        self.get_logger().info(f"tile_topic : {self.tile_topic}")

    # --------------------------------------------------
    # callbacks
    # --------------------------------------------------
    def color_callback(self, msg: Image):
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"color_callback failed: {e}")

    def depth_callback(self, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            # uint16 depth(mm) 기준
            if depth.dtype != np.uint16:
                depth = np.asarray(depth)

            self.depth_image = depth
        except Exception as e:
            self.get_logger().warn(f"depth_callback failed: {e}")

    def tile_array_callback(self, msg: TileArray):
        self.latest_tile_array = msg
        self.latest_tile_stamp = time.time()
        self.get_logger().info(f"[tile_array] received {len(msg.tiles)} tiles")

    def on_timer(self):
        try:
            if self.color_image is None:
                self.get_logger().warn("color image not ready")
                return
            if self.depth_image is None:
                self.get_logger().warn("depth image not ready")
                return

            self._show_debug_image()
        except Exception as e:
            self.get_logger().warn(f"debug draw failed: {e}")

    # --------------------------------------------------
    # helpers
    # --------------------------------------------------
    def _depth_hw(self):
        if self.depth_image is None:
            raise RuntimeError("depth_image is None")
        return self.depth_image.shape[:2]

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

    def _get_color_image_for_debug(self):
        if self.color_image is None:
            return None
        return self.color_image.copy()

    def _get_depth_vis_for_debug(self):
        if self.depth_image is None:
            return None

        depth = self.depth_image.astype(np.float32)

        valid = (depth >= self.min_mm) & (depth <= self.max_mm)

        vis = np.zeros(depth.shape, dtype=np.uint8)

        # 프레임마다 min/max 다시 잡지 말고 고정 범위 사용
        scale = 255.0 / float(self.max_mm - self.min_mm)

        vis[valid] = np.clip(
            (depth[valid] - float(self.min_mm)) * scale,
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

        tile_crop_rows = []

        tile_count = 0 if self.latest_tile_array is None else len(self.latest_tile_array.tiles)
        self.get_logger().info(f"[debug] tile_count={tile_count}")

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


def main(args=None):
    rclpy.init(args=args)
    node = DepthRoiDebugNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()