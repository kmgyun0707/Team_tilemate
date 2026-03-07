#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2


class CenterDepthFilterNode(Node):
    def __init__(self):
        super().__init__('center_depth_filter_node')

        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.image_callback,
            10
        )

        self.window_name = 'Depth Heatmap'
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        self.get_logger().info('CenterDepthFilterNode started.')

    def image_callback(self, msg: Image):
        if msg.encoding != '16UC1':
            self.get_logger().error(f'Unsupported encoding: {msg.encoding}')
            return

        try:
            depth_image = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        except Exception as e:
            self.get_logger().error(f'Failed to reshape depth image: {e}')
            return

        cy = msg.height // 2
        cx = msg.width // 2

        half = 2  # 5x5
        y1 = max(0, cy - half)
        y2 = min(msg.height, cy + half + 1)
        x1 = max(0, cx - half)
        x2 = min(msg.width, cx + half + 1)

        center_patch = depth_image[y1:y2, x1:x2]

        valid_depths = center_patch[center_patch > 0]
        valid_depths = valid_depths[(valid_depths >= 200) & (valid_depths <= 5000)]

        filtered_depth = None
        filtered_median = None

        if valid_depths.size > 0:
            median_depth = np.median(valid_depths)
            mean_depth = np.mean(valid_depths)

            inliers = valid_depths[np.abs(valid_depths - median_depth) <= 100]

            if inliers.size > 0:
                filtered_depth = float(np.mean(inliers))
                filtered_median = float(np.median(inliers))
            else:
                filtered_depth = float(mean_depth)
                filtered_median = float(median_depth)

            self.get_logger().info(
                f'center=({cx},{cy}), patch_shape={center_patch.shape}, '
                f'valid={valid_depths.size}, '
                f'filtered_mean={filtered_depth:.1f} mm, '
                f'filtered_median={filtered_median:.1f} mm'
            )
            self.get_logger().info(
                f'depth min={depth_image[depth_image>0].min() if np.any(depth_image>0) else -1}, '
                f'max={depth_image.max()}, '
                f'center_raw={depth_image[cy, cx]}'
            )

        # ----------------------------
        # GUI용 depth 히트맵 생성
        # ----------------------------
        vis_depth = depth_image.copy()

        # 0값은 보기 쉽게 0으로 두고, 나머지는 시각화 범위로 클리핑
        vis_min = 300    # mm
        vis_max = 3000   # mm

        vis_depth = np.clip(vis_depth, vis_min, vis_max)

        # 0값 마스크
        zero_mask = (depth_image == 0)

        # 0~255로 정규화
        vis_norm = ((vis_depth.astype(np.float32) - vis_min) / (vis_max - vis_min) * 255.0)
        vis_norm = np.clip(vis_norm, 0, 255).astype(np.uint8)

        # 컬러맵 적용
        heatmap = cv2.applyColorMap(vis_norm, cv2.COLORMAP_JET)

        # invalid depth(0)는 검정색으로 표시
        heatmap[zero_mask] = (0, 0, 0)

        # ----------------------------
        # 가운데 점 / 5x5 박스 / 텍스트 표시
        # ----------------------------
        cv2.circle(heatmap, (cx, cy), 4, (255, 255, 255), -1)
        cv2.rectangle(heatmap, (x1, y1), (x2 - 1, y2 - 1), (255, 255, 255), 1)

        if filtered_median is not None:
            text1 = f'Center: ({cx}, {cy})'
            text2 = f'Depth: {filtered_median:.1f} mm ({filtered_median / 1000.0:.3f} m)'
        else:
            text1 = f'Center: ({cx}, {cy})'
            text2 = 'Depth: invalid'

        cv2.putText(
            heatmap, text1, (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA
        )
        cv2.putText(
            heatmap, text2, (20, 60),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA
        )

        # 가운데 점 옆에도 거리 표시
        label_x = min(cx + 10, msg.width - 200)
        label_y = max(cy - 10, 20)

        if filtered_median is not None:
            center_label = f'{filtered_median:.0f} mm'
        else:
            center_label = 'invalid'

        cv2.putText(
            heatmap, center_label, (label_x, label_y),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA
        )

        cv2.imshow(self.window_name, heatmap)
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CenterDepthFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()