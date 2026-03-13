#!/usr/bin/env python3
import os
import math
import time
import threading

import cv2
import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

from custom_tile_msgs.msg import TileArray, InspectionResult, InspectionResultArray
from tilemate_msgs.action import PatternInspect


class PatternInspectActionServer(Node):
    def __init__(self):
        super().__init__("pattern_inspect_action_server")

        self.cb_group = ReentrantCallbackGroup()
        self.bridge = CvBridge()

        # --------------------------------------------------
        # pattern reference setup
        # --------------------------------------------------
        package_name = "tilemate_main"
        pkg_share_dir = get_package_share_directory(package_name)
        img_base_dir = os.path.join(pkg_share_dir, "rule_based_img")

        self.ref_image_paths = {
            f"pattern_{i}": os.path.join(img_base_dir, f"ref_pattern_{i}.jpg")
            for i in range(1, 10)
        }

        self.safe_zones = {}
        kernel = np.ones((11, 11), np.uint8)
        self.default_pixel_threshold = 60
        self.default_target_frames = 60
        self.default_match_dist_threshold = 50.0

        for pattern_name, img_path in self.ref_image_paths.items():
            if not os.path.exists(img_path):
                self.get_logger().warn(
                    f"🚨 기준 이미지 없음: {img_path} ({pattern_name} 검사 불가)"
                )
                continue

            ref_img = cv2.imread(img_path)
            if ref_img is None:
                self.get_logger().warn(
                    f"🚨 기준 이미지 로드 실패: {img_path} ({pattern_name} 검사 불가)"
                )
                continue

            ref_edges = self.get_canny_edges(ref_img)
            self.safe_zones[pattern_name] = cv2.dilate(ref_edges, kernel, iterations=1)

        self.get_logger().info(
            f"✅ {len(self.safe_zones)}개 패턴의 안전지대 로드 완료"
        )

        # --------------------------------------------------
        # runtime state
        # --------------------------------------------------
        self._lock = threading.Lock()
        self._active_goal_handle = None
        self._inspect_running = False

        self.current_frame_count = 0
        self.target_frames = self.default_target_frames
        self.pixel_threshold = self.default_pixel_threshold
        self.match_dist_threshold = self.default_match_dist_threshold
        self.tracked_tiles = []
        self._last_header = None
        self._done_event = threading.Event()
        self._result_summary = {
            "success": False,
            "message": "idle",
            "total_tiles": 0,
            "defective_tiles": 0,
        }

        # --------------------------------------------------
        # ros interfaces
        # --------------------------------------------------
        self.tile_sub = self.create_subscription(
            TileArray,
            "/yolo/tile_array",
            self.inference_callback,
            10,
            callback_group=self.cb_group,
        )

        self.web_pub = self.create_publisher(
            InspectionResultArray,
            "/web/inspection_results",
            10,
        )

        self.debug_pub = self.create_publisher(
            Image,
            "/anomaly/debug_rule_based",
            10,
        )

        self._action_server = ActionServer(
            self,
            PatternInspect,
            "tile/pattern_inspect",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group,
        )

        self.get_logger().info("✅ PatternInspectActionServer ready")

    # --------------------------------------------------
    # action callbacks
    # --------------------------------------------------
    def goal_callback(self, goal_request):
        with self._lock:
            if self._inspect_running:
                self.get_logger().warn("[PATTERN] reject: already running")
                return GoalResponse.REJECT

        if goal_request.target_frames < 1:
            self.get_logger().warn("[PATTERN] reject: target_frames < 1")
            return GoalResponse.REJECT

        self.get_logger().info(
            f"[PATTERN] goal accepted: "
            f"target_frames={goal_request.target_frames}, "
            f"pixel_threshold={goal_request.pixel_threshold}, "
            f"match_dist_threshold={goal_request.match_dist_threshold}"
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().warn("[PATTERN] cancel request received")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        result = PatternInspect.Result()

        with self._lock:
            self._active_goal_handle = goal_handle
            self._inspect_running = True
            self._done_event.clear()

            req = goal_handle.request
            self.target_frames = (
                int(req.target_frames)
                if int(req.target_frames) > 0
                else self.default_target_frames
            )
            self.pixel_threshold = (
                int(req.pixel_threshold)
                if int(req.pixel_threshold) > 0
                else self.default_pixel_threshold
            )
            self.match_dist_threshold = (
                float(req.match_dist_threshold)
                if float(req.match_dist_threshold) > 0.0
                else self.default_match_dist_threshold
            )

            self.reset_buffer_locked()

            self._result_summary = {
                "success": False,
                "message": "running",
                "total_tiles": 0,
                "defective_tiles": 0,
            }

        self.publish_feedback(
            goal_handle,
            current_frame=0,
            progress=0.0,
            state="started",
        )

        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    with self._lock:
                        self._inspect_running = False
                        self._active_goal_handle = None
                        self.reset_buffer_locked()

                    goal_handle.canceled()
                    result.success = False
                    result.message = "canceled"
                    result.total_tiles = 0
                    result.defective_tiles = 0
                    return result

                if self._done_event.wait(timeout=0.1):
                    break

            with self._lock:
                summary = dict(self._result_summary)
                self._inspect_running = False
                self._active_goal_handle = None
                self.reset_buffer_locked()

            if not summary["success"]:
                goal_handle.abort()
            else:
                goal_handle.succeed()

            result.success = bool(summary["success"])
            result.message = str(summary["message"])
            result.total_tiles = int(summary["total_tiles"])
            result.defective_tiles = int(summary["defective_tiles"])
            return result

        except Exception as e:
            self.get_logger().error(f"[PATTERN] execute failed: {e}")

            with self._lock:
                self._inspect_running = False
                self._active_goal_handle = None
                self.reset_buffer_locked()

            goal_handle.abort()
            result.success = False
            result.message = f"exception: {e}"
            result.total_tiles = 0
            result.defective_tiles = 0
            return result

    # --------------------------------------------------
    # image preprocessing
    # --------------------------------------------------
    def get_canny_edges(self, cv_img):
        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))
        gray_clahe = clahe.apply(blurred)
        edges = cv2.Canny(gray_clahe, 35, 95)

        h, w = edges.shape
        cv2.rectangle(edges, (0, 0), (w - 1, h - 1), 0, thickness=5)
        return edges

    # --------------------------------------------------
    # subscriber callback
    # --------------------------------------------------
    def inference_callback(self, msg: TileArray):
        with self._lock:
            if not self._inspect_running:
                return

            if self.current_frame_count >= self.target_frames:
                return

        try:
            local_debug_images = []

            for tile in msg.tiles:
                current_pattern = tile.pattern_name

                if current_pattern not in self.safe_zones:
                    self.get_logger().warn(
                        f"⚠️ 등록되지 않은 패턴({current_pattern}) 감지! 검사 생략."
                    )
                    continue

                target_safe_zone = self.safe_zones[current_pattern]

                curr_img = self.bridge.imgmsg_to_cv2(
                    tile.cropped_image,
                    desired_encoding="bgr8",
                )
                curr_img = cv2.resize(
                    curr_img,
                    (target_safe_zone.shape[1], target_safe_zone.shape[0]),
                )

                curr_edges = self.get_canny_edges(curr_img)
                defect_edges = cv2.subtract(curr_edges, target_safe_zone)

                defect_pixel_count = int(np.sum(defect_edges > 0))
                is_defect = bool(defect_pixel_count > self.pixel_threshold)

                matched = False
                for tt in self.tracked_tiles:
                    dist = math.hypot(
                        tt["pose"].x - tile.pose.x,
                        tt["pose"].y - tile.pose.y,
                    )

                    if dist < self.match_dist_threshold:
                        tt["crack_history"].append(defect_pixel_count)
                        tt["vote_history"].append(is_defect)
                        tt["pose"] = tile.pose
                        matched = True
                        break

                if not matched:
                    self.tracked_tiles.append(
                        {
                            "pose": tile.pose,
                            "pattern_name": current_pattern,
                            "width": tile.width,
                            "height": tile.height,
                            "crack_history": [defect_pixel_count],
                            "vote_history": [is_defect],
                        }
                    )

                local_debug_images.append((curr_img, defect_edges))

            self.current_frame_count += 1
            self._last_header = msg.header

            goal_handle = self._active_goal_handle
            current_frame = self.current_frame_count
            target_frames = self.target_frames

            if goal_handle is not None:
                progress = float(current_frame) / float(max(1, target_frames))
                self.publish_feedback(
                    goal_handle,
                    current_frame=current_frame,
                    progress=progress,
                    state="collecting_frames",
                )

            self.get_logger().info(
                f"⏳ 패턴 검사 데이터 누적 중... "
                f"({self.current_frame_count}/{self.target_frames} 프레임)"
            )

            if self.current_frame_count == self.target_frames:
                for curr_img, defect_edges in local_debug_images:
                    curr_img_debug = curr_img.copy()
                    curr_img_debug[defect_edges > 0] = [0, 0, 255]
                    debug_msg = self.bridge.cv2_to_imgmsg(
                        curr_img_debug,
                        encoding="bgr8",
                    )
                    self.debug_pub.publish(debug_msg)

            if self.current_frame_count >= self.target_frames:
                total_tiles, defective_tiles = self.publish_final_results(msg.header)

                self._result_summary = {
                    "success": True,
                    "message": "pattern_inspection_complete",
                    "total_tiles": total_tiles,
                    "defective_tiles": defective_tiles,
                }

                if goal_handle is not None:
                    self.publish_feedback(
                        goal_handle,
                        current_frame=self.current_frame_count,
                        progress=1.0,
                        state="completed",
                    )

                self._done_event.set()

        except Exception as e:
            self.get_logger().error(f"🚨 패턴 검사 추론 중 에러: {e}")
            self._result_summary = {
                "success": False,
                "message": f"exception: {e}",
                "total_tiles": 0,
                "defective_tiles": 0,
            }
            self._done_event.set()

    # --------------------------------------------------
    # result publish
    # --------------------------------------------------
    def publish_final_results(self, header):
        result_array_msg = InspectionResultArray()
        result_array_msg.header = header

        self.get_logger().info("=========================================")
        self.get_logger().info(
            f"🎯 {self.target_frames}프레임 다수결 패턴 검사 완료! (결과 웹 전송)"
        )

        defective_count = 0

        for idx, tt in enumerate(self.tracked_tiles):
            if len(tt["vote_history"]) < (self.target_frames / 2):
                continue

            total_votes = len(tt["vote_history"])
            defect_votes = sum(tt["vote_history"])
            avg_cracks = sum(tt["crack_history"]) / total_votes

            final_is_defect = bool(defect_votes >= (total_votes / 2.0))

            res_msg = InspectionResult()
            res_msg.tile_id = idx + 1
            res_msg.pattern_name = tt["pattern_name"]
            res_msg.pose = tt["pose"]
            res_msg.width = tt["width"]
            res_msg.height = tt["height"]
            res_msg.is_defective = final_is_defect
            res_msg.defect_type = "Crack" if final_is_defect else "None"

            result_array_msg.results.append(res_msg)

            if final_is_defect:
                defective_count += 1

            status = "🚨 최종 폐기" if final_is_defect else "✅ 최종 정상"
            self.get_logger().info(
                f"[{tt['pattern_name']}] 타일 {idx + 1}: {status} | "
                f"불량 판정: {defect_votes}/{total_votes}회 "
                f"(평균 크랙: {avg_cracks:.1f}px)"
            )

        self.get_logger().info("=========================================")

        result_array_msg.total_tiles = len(result_array_msg.results)

        if result_array_msg.total_tiles > 0:
            self.web_pub.publish(result_array_msg)

        return result_array_msg.total_tiles, defective_count

    # --------------------------------------------------
    # utils
    # --------------------------------------------------
    def publish_feedback(self, goal_handle, current_frame: int, progress: float, state: str):
        fb = PatternInspect.Feedback()
        fb.current_frame = int(current_frame)
        fb.progress = float(progress)
        fb.state = str(state)
        goal_handle.publish_feedback(fb)

    def reset_buffer_locked(self):
        self.current_frame_count = 0
        self.tracked_tiles = []
        self._last_header = None

    # 외부에서 락 없이 부를 경우 대비용
    def reset_buffer(self):
        with self._lock:
            self.reset_buffer_locked()


def main(args=None):
    rclpy.init(args=args)
    node = PatternInspectActionServer()
    ex = MultiThreadedExecutor()
    ex.add_node(node)

    try:
        ex.spin()
    except KeyboardInterrupt:
        node.get_logger().info("종료 중...")
    finally:
        try:
            ex.remove_node(node)
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()