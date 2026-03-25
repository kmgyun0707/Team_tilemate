#!/usr/bin/env python3
import os
import math
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

from custom_tile_msgs.msg import TileArray
from tilemate_msgs.action import PatternInspect
from tilemate_msgs.msg import PatternScore


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
                    f"기준 이미지 없음: {img_path} ({pattern_name} 검사 불가)"
                )
                continue

            ref_img = cv2.imread(img_path)
            if ref_img is None:
                self.get_logger().warn(
                    f"기준 이미지 로드 실패: {img_path} ({pattern_name} 검사 불가)"
                )
                continue

            ref_edges = self.get_canny_edges(ref_img)
            self.safe_zones[pattern_name] = cv2.dilate(ref_edges, kernel, iterations=1)

        

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
            "pattern_scores": [],
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

        self.get_logger().info("\033[94m [3/3] [PATTERN_INSPECT] initialize Done!\033[0m")

    # --------------------------------------------------
    # action callbacks
    # --------------------------------------------------
    def goal_callback(self, goal_request):
        with self._lock:
            if self._inspect_running:
                self.get_logger().warn("[PATTERN] reject: already running")
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
                "pattern_scores": [],
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
                    return result

                if self._done_event.wait(timeout=0.1):
                    break

            with self._lock:
                summary = dict(self._result_summary)
                self._inspect_running = False
                self._active_goal_handle = None
                self.reset_buffer_locked()

            self.get_logger().info(
                f"[PATTERN] result build start: success={summary['success']}, "
                f"count={len(summary.get('pattern_scores', []))}"
            )

            result.success = bool(summary["success"])
            result.message = str(summary["message"])

            for i, ps in enumerate(summary.get("pattern_scores", []), start=1):
                self.get_logger().info(
                    f"[PATTERN] pack score {i}: "
                    f"name={ps['pattern_name']}, score={ps['anomaly_score']}"
                )
                score_msg = PatternScore()
                score_msg.tile_id = int(ps["tile_id"])
                score_msg.pattern_name = str(ps["pattern_name"])
                score_msg.anomaly_score = float(ps["anomaly_score"])
                result.pattern_scores.append(score_msg)

            if not summary["success"]:
                self.get_logger().warn("[PATTERN] goal abort")
                goal_handle.abort()
            else:
                self.get_logger().info("[PATTERN] goal succeed")
                goal_handle.succeed()

            self.get_logger().info("[PATTERN] returning action result")
            return result

        except Exception as e:
            self.get_logger().error(f"[PATTERN] execute failed: {e}")

            with self._lock:
                self._inspect_running = False
                self._active_goal_handle = None
                self.reset_buffer_locked()

            try:
                goal_handle.abort()
            except Exception:
                pass

            result.success = False
            result.message = f"exception: {e}"
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
                        f"등록되지 않은 패턴({current_pattern}) 감지! 검사 생략."
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
                            "pose": tile.pose,  # tracking 내부용
                            "pattern_name": current_pattern,
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
                f"패턴 검사 데이터 누적 중... "
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
                pattern_scores = self.publish_final_results()

                self._result_summary = {
                    "success": True,
                    "message": "pattern_inspection_complete",
                    "pattern_scores": pattern_scores,
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
            self.get_logger().error(f"패턴 검사 추론 중 에러: {e}")
            self._result_summary = {
                "success": False,
                "message": f"exception: {e}",
                "pattern_scores": [],
            }
            self._done_event.set()

    # --------------------------------------------------
    # result build
    # --------------------------------------------------
    def publish_final_results(self):
        pattern_scores = []

        self.get_logger().info("=========================================")
        self.get_logger().info(
            f"{self.target_frames}프레임 다수결 패턴 검사 완료"
        )

        for idx, tt in enumerate(self.tracked_tiles):
            if len(tt["vote_history"]) < (self.target_frames / 2):
                continue

            total_votes = len(tt["vote_history"])
            avg_cracks = sum(tt["crack_history"]) / total_votes

            anomaly_score = min(
                1.0,
                avg_cracks / float(max(1, self.pixel_threshold))
            )

            pattern_scores.append(
                {
                    "tile_id": idx + 1,
                    "pattern_name": tt["pattern_name"],
                    "anomaly_score": float(anomaly_score),
                }
            )

            self.get_logger().info(
                f"[{tt['pattern_name']}] 타일 {idx + 1} | "
                f"avg_cracks={avg_cracks:.1f}px | "
                f"anomaly_score={anomaly_score:.2f}"
            )

        self.get_logger().info("=========================================")

        return pattern_scores

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