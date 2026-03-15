#!/usr/bin/env python3
import os
import math
import threading

import cv2
import numpy as np
import rclpy
import torch

from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from torchvision import transforms
from anomalib.models import Patchcore

from custom_tile_msgs.msg import TileArray
from tilemate_msgs.action import PatternInspect
from tilemate_msgs.msg import PatternScore


class PatternInspectActionServer(Node):
    def __init__(self):
        super().__init__("pattern_inspect_action_server")

        # 액션 execute 루프와 이미지 콜백이 동시에 접근하는 상태를 보호하기 위한 그룹/브릿지
        self.cb_group = ReentrantCallbackGroup()
        self.bridge = CvBridge()

        # --------------------------------------------------
        # anomalib setup
        # --------------------------------------------------
        # rule-based reference image 방식 대신 Patchcore 체크포인트를 로드해
        # 액션형 패턴 검사 서버 내부에서 직접 추론을 수행한다.
        package_name = "tilemate_main"
        pkg_share_dir = get_package_share_directory(package_name)

        # 런치에서 쉽게 바꿀 수 있도록 모든 핵심 값은 파라미터로 받는다.
        self.declare_parameter(
            "ckpt_filename",
            "dataset_1280_type6_patchcore.ckpt",
        )
        self.declare_parameter("target_pattern", "pattern_5")
        self.declare_parameter("use_canny_filter", False)
        self.declare_parameter("anomaly_threshold", -1.0)

        ckpt_filename = str(self.get_parameter("ckpt_filename").value)
        self.target_pattern = str(self.get_parameter("target_pattern").value)
        self.use_canny_filter = bool(self.get_parameter("use_canny_filter").value)

        ckpt_path = os.path.join(pkg_share_dir, "resource", ckpt_filename)
        if not os.path.exists(ckpt_path):
            raise FileNotFoundError(
                f"anomalib checkpoint not found: {ckpt_path}"
            )

        # GPU 사용 가능 시 cuda, 아니면 cpu로 자동 폴백
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model = Patchcore.load_from_checkpoint(ckpt_path)
        self.model.eval()
        self.model.to(self.device)

        # 임계값 우선순위
        # 1) launch/CLI에서 anomaly_threshold 지정값(>0)
        # 2) 체크포인트 내부 image_threshold
        # 3) 하드코딩 기본값 0.65
        self.anomaly_threshold = 0.65
        threshold_param = float(self.get_parameter("anomaly_threshold").value)
        if threshold_param > 0.0:
            self.anomaly_threshold = threshold_param
        else:
            try:
                if hasattr(self.model, "image_threshold"):
                    self.anomaly_threshold = self.model.image_threshold.value.item()
                elif hasattr(self.model, "metrics") and hasattr(self.model.metrics, "image_threshold"):
                    self.anomaly_threshold = self.model.metrics.image_threshold.value.item()
            except Exception:
                pass

        # Tile cropped image는 크기가 제각각이므로 모델 입력 크기를 고정한다.
        self.transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((512, 512), antialias=True),
            transforms.ToTensor(),
        ])

        self.default_target_frames = 60
        self.default_match_dist_threshold = 50.0

        # --------------------------------------------------
        # runtime state
        # --------------------------------------------------
        # _inspect_running: 동시에 goal 2개가 겹치지 않도록 단일 실행을 보장
        # _done_event: 프레임 수집 완료 시 execute 루프를 깨우는 신호
        self._lock = threading.Lock()
        self._active_goal_handle = None
        self._inspect_running = False

        self.current_frame_count = 0
        self.target_frames = self.default_target_frames
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
        # 입력: YOLO 타일 배열(크롭 이미지 + pose + pattern)
        self.tile_sub = self.create_subscription(
            TileArray,
            "/yolo/tile_array",
            self.inference_callback,
            10,
            callback_group=self.cb_group,
        )

        # 디버그용: 모델 입력 이미지(필터 적용 여부 확인)와 히트맵 오버레이
        self.debug_pub = self.create_publisher(
            Image,
            "/anomaly/debug_canny",
            10,
        )
        self.heatmap_pub = self.create_publisher(
            Image,
            "/anomaly/heatmap",
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
        self.get_logger().info(
            f"[PATTERN] anomalib ready | device={self.device}, target_pattern={self.target_pattern or 'ALL'}, "
            f"threshold={self.anomaly_threshold:.4f}, use_canny_filter={self.use_canny_filter}"
        )

    # --------------------------------------------------
    # action callbacks
    # --------------------------------------------------
    def goal_callback(self, goal_request):
        # 액션 서버는 한 번에 하나의 검사만 수행한다.
        # (프레임 누적 버퍼를 공유하므로 병렬 goal은 reject)
        with self._lock:
            if self._inspect_running:
                self.get_logger().warn("[PATTERN] reject: already running")
                return GoalResponse.REJECT

        self.get_logger().info(
            f"[PATTERN] goal accepted: "
            f"target_frames={goal_request.target_frames}, "
            f"match_dist_threshold={goal_request.match_dist_threshold}"
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().warn("[PATTERN] cancel request received")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        result = PatternInspect.Result()

        with self._lock:
            # goal 시작 시 런타임 파라미터를 action request 값으로 덮어쓴다.
            # (기존 TaskManager/InspectActionServer와의 계약을 유지)
            self._active_goal_handle = goal_handle
            self._inspect_running = True
            self._done_event.clear()

            req = goal_handle.request
            self.target_frames = (
                int(req.target_frames)
                if int(req.target_frames) > 0
                else self.default_target_frames
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
                    # cancel 요청 시 버퍼/상태를 즉시 비우고 종료
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
                # 콜백에서 만든 요약본을 action result로 패킹한다.
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
                # tile_id / pattern_name / anomaly_score 형식은
                # 상위 InspectActionServer merge 로직과 맞춘다.
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

    def apply_industry_filter(self, cv_img):
        # 고주파 노이즈 완화 + 대비 보정으로 금속/타일 표면 질감 안정화
        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        filtered = cv2.bilateralFilter(gray, 9, 75, 75)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        clahe_img = clahe.apply(filtered)
        return cv2.cvtColor(clahe_img, cv2.COLOR_GRAY2RGB)

    def _extract_pred_score(self, output):
        # anomalib 반환 타입(dict/tuple/object)이 버전에 따라 달라질 수 있어
        # 공통 추출 루틴으로 단일 float 점수로 정규화한다.
        if hasattr(output, "pred_score"):
            pred_score = output.pred_score
        elif isinstance(output, dict):
            pred_score = output.get("pred_score", 0.0)
        elif isinstance(output, tuple) and len(output) > 1:
            pred_score = output[1]
        else:
            pred_score = 0.0

        if isinstance(pred_score, torch.Tensor):
            if pred_score.numel() == 0:
                return 0.0
            pred_score = pred_score.detach().float().mean().cpu().item()

        return float(pred_score)

    def _extract_anomaly_map(self, output):
        # heatmap 시각화를 위한 anomaly_map을 안전하게 추출/정규화
        if hasattr(output, "anomaly_map"):
            anomaly_map = output.anomaly_map
        elif isinstance(output, dict):
            anomaly_map = output.get("anomaly_map", None)
        elif isinstance(output, tuple) and len(output) > 0:
            anomaly_map = output[0]
        else:
            anomaly_map = None

        if anomaly_map is None:
            return None

        if isinstance(anomaly_map, torch.Tensor):
            anomaly_map = anomaly_map.detach().cpu().numpy()

        anomaly_map = np.squeeze(np.array(anomaly_map, dtype=np.float32))
        return anomaly_map

    # --------------------------------------------------
    # subscriber callback
    # --------------------------------------------------
    def inference_callback(self, msg: TileArray):
        # 액션 진행 중일 때만 프레임을 누적한다.
        with self._lock:
            if not self._inspect_running:
                return

            if self.current_frame_count >= self.target_frames:
                return

        try:
            local_debug_images = []
            inspected_any = False

            for tile in msg.tiles:
                current_pattern = tile.pattern_name

                # target_pattern이 설정된 경우 해당 패턴만 수집해
                # 불필요한 오검출/계산량을 줄인다.
                if self.target_pattern and current_pattern != self.target_pattern:
                    continue

                inspected_any = True

                cv_img = self.bridge.imgmsg_to_cv2(
                    tile.cropped_image,
                    desired_encoding="bgr8",
                )
                h, w = cv_img.shape[:2]

                if self.use_canny_filter:
                    model_input_img = self.apply_industry_filter(cv_img)
                else:
                    model_input_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)

                input_tensor = self.transform(model_input_img).unsqueeze(0).to(self.device)
                with torch.no_grad():
                    output = self.model(input_tensor)

                pred_score = self._extract_pred_score(output)
                anomaly_map = self._extract_anomaly_map(output)
                is_defect = bool(pred_score > self.anomaly_threshold)

                # 프레임 간 동일 타일 매칭: pose(x, y) 거리 기반 nearest-like 누적
                matched = False
                for tt in self.tracked_tiles:
                    dist = math.hypot(
                        tt["pose"].x - tile.pose.x,
                        tt["pose"].y - tile.pose.y,
                    )

                    if dist < self.match_dist_threshold:
                        tt["score_history"].append(pred_score)
                        tt["vote_history"].append(is_defect)
                        tt["pose"] = tile.pose
                        matched = True
                        break

                if not matched:
                    self.tracked_tiles.append(
                        {
                            "pose": tile.pose,
                            "pattern_name": current_pattern,
                            "score_history": [pred_score],
                            "vote_history": [is_defect],
                        }
                    )

                local_debug_images.append((cv_img, model_input_img, anomaly_map, w, h))

            if not inspected_any:
                # 이번 프레임에 타깃 패턴이 없으면 프레임 카운트를 올리지 않는다.
                return

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
                # 마지막 프레임에서만 디버그/히트맵 발행 (대역폭 절감)
                for cv_img, model_input_img, anomaly_map, w, h in local_debug_images:
                    if anomaly_map is not None:
                        norm_max = self.anomaly_threshold * 2.0
                        a_map_norm = np.clip(anomaly_map / (norm_max + 1e-5), 0, 1)
                        a_map_resized = cv2.resize(
                            (a_map_norm * 255).astype(np.uint8),
                            (int(w), int(h)),
                            interpolation=cv2.INTER_CUBIC,
                        )
                        heatmap = cv2.applyColorMap(a_map_resized, cv2.COLORMAP_JET)
                        overlay = cv2.addWeighted(cv_img, 0.5, heatmap, 0.5, 0)
                        self.heatmap_pub.publish(
                            self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
                        )

                    debug_bgr = (
                        cv2.cvtColor(model_input_img, cv2.COLOR_RGB2BGR)
                        if model_input_img is not None and len(model_input_img.shape) == 3
                        else cv_img
                    )
                    debug_msg = self.bridge.cv2_to_imgmsg(debug_bgr, encoding="bgr8")
                    self.debug_pub.publish(debug_msg)

            if self.current_frame_count >= self.target_frames:
                # 누적 프레임 달성 시 즉시 최종 점수 생성 후 execute 루프에 완료 신호 전달
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
            # 관측 횟수가 너무 적은 타일은 노이즈로 판단해 제외
            if len(tt["vote_history"]) < (self.target_frames / 2):
                continue

            total_votes = len(tt["vote_history"])
            # anomaly_score는 pred_score 평균값으로 정의
            # (상위 merge 단계에서 pattern_name별 단일 점수가 필요)
            avg_score = sum(tt["score_history"]) / total_votes

            pattern_scores.append(
                {
                    "tile_id": idx + 1,
                    "pattern_name": tt["pattern_name"],
                    "anomaly_score": float(avg_score),
                }
            )

            self.get_logger().info(
                f"[{tt['pattern_name']}] 타일 {idx + 1} | "
                f"avg_score={avg_score:.4f} | "
                f"threshold={self.anomaly_threshold:.4f}"
            )

        self.get_logger().info("=========================================")

        return pattern_scores

    # --------------------------------------------------
    # utils
    # --------------------------------------------------
    def publish_feedback(self, goal_handle, current_frame: int, progress: float, state: str):
        # TaskManager/InspectActionServer가 진행률을 읽을 수 있도록
        # 프레임 기반 피드백을 지속 발행한다.
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