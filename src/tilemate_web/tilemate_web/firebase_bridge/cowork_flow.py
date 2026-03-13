"""
firebase_bridge/cowork_flow.py
협동작업 흐름 처리:
  - 타일 파지 대기 (WAIT_HUMAN_TAKE)
  - 시멘트 도포 대기 (WAIT_CEMENT_DONE)
  - STT 트리거 처리
"""

import time
from std_msgs.msg import Bool
from . import tts_helper


PICK_KEYWORDS    = ["잡았어", "잡았다", "집었어", "잡았음", "타일 잡았어", "집었다"]
CEMENT_KEYWORDS  = ["발랐어", "발랐다", "다 발랐어", "시멘트 다 발랐어", "완료", "됐어", "다됐어", "끝났어", "끝났다"]


def _wait_for_keyword(stt, keywords: list, log_prefix: str, firebase_ref, logger) -> bool:
    """
    STT로 음성을 반복 수신하여 키워드 감지 시 True 반환.
    STT가 None이면 5초 대기 후 자동 진행.
    Firebase robot_status/manual_confirm 이 True 이면 버튼 확인으로 즉시 진행.
    """
    if stt is None:
        logger.warn(f"[{log_prefix}] STT 없음. 5초 후 자동 진행합니다.")
        time.sleep(5.0)
        return True

    # 수동 확인 플래그 초기화
    firebase_ref.update({"manual_confirm": False})

    first_attempt = True
    while True:
        # 버튼 수동 확인 체크
        try:
            snap = firebase_ref.get()
            if snap and snap.get("manual_confirm") is True:
                firebase_ref.update({"manual_confirm": False, "stt_mic_state": ""})
                logger.info(f"[{log_prefix}] 수동 확인 버튼 → 진행")
                return True
        except Exception as e:
            logger.warn(f"[{log_prefix}] manual_confirm 체크 오류 (무시): {e}")

        try:
            logger.info(f"[{log_prefix}] 🎙 음성 대기 중...")
            if first_attempt:
                firebase_ref.update({"stt_mic_state": "calibrating"})
            text = stt.speech2text()
            first_attempt = False
            logger.info(f"[{log_prefix}] STT 결과: '{text}'")

            if any(kw in text for kw in keywords):
                firebase_ref.update({"stt_mic_state": ""})
                return True
            else:
                logger.info(f"[{log_prefix}] 키워드 미감지. 재청취...")
                firebase_ref.update({"stt_mic_state": "retry"})
        except Exception as e:
            logger.error(f"[{log_prefix}] STT 오류: {e}")
            firebase_ref.update({"stt_mic_state": "retry"})
            first_attempt = False
            time.sleep(1.0)


class CoworkFlowMixin:
    """
    FirebaseBridgeNode에 믹스인으로 사용.
    self.ref, self._stt, self._pub_human_take, self._pub_cement_done,
    self._publish_reliable, self.get_logger() 에 접근한다고 가정.
    """

    def _cement_pick_flow(self):
        """
        WAIT_HUMAN_TAKE 진입 시 실행:
        1. 웹 팝업 → waiting_pick
        2. TTS: "타일을 잡아주세요"
        3. STT: PICK_KEYWORDS 감지
        4. ROS publish: /dsr01/cowork/human_take_confirm (Bool True)
        """
        try:
            self.ref.update({
                "state": "타일 파지 대기 중 - 타일을 잡으면 '타일 잡았어' 라고 말해주세요",
                "cement_state": "waiting_pick",
            })
            self.get_logger().info("[CEMENT] 1단계: 타일 파지 대기")

            self.ref.update({"stt_mic_state": "tts_speaking"})
            tts_helper.speak("타일을 잡아주세요", firebase_ref=self.ref, logger=self.get_logger())
            time.sleep(0.5)

            _wait_for_keyword(self._stt, PICK_KEYWORDS, "CEMENT/PICK", self.ref, self.get_logger())

            msg = Bool()
            msg.data = True
            self._publish_reliable(self._pub_human_take, msg, retries=3)
            self.get_logger().info("[CEMENT] human_take_confirm 토픽 전송 완료")
            self.ref.update({"state": "타일 내려놓는 중", "cement_state": "tile_release", "stt_mic_state": ""})

        except Exception as e:
            self.get_logger().error(f"[CEMENT] _cement_pick_flow 오류: {e}")
        finally:
            self._cement_waiting = False

    def _cement_done_flow(self):
        """
        WAIT_CEMENT_DONE 진입 시 실행:
        1. 웹 팝업 → waiting_cement
        2. TTS: "시멘트를 발라주세요"
        3. STT: CEMENT_KEYWORDS 감지
        4. TTS: "타일을 두세요"
        5. ROS publish: /dsr01/cowork/cement_done (Bool True)
        """
        try:
            self.ref.update({
                "state": "시멘트 도포 대기 중 - 다 바르면 '시멘트 다 발랐어' 라고 말해주세요",
                "cement_state": "waiting_cement",
                "stt_mic_state": "",
            })
            self.get_logger().info("[CEMENT] 2단계: 시멘트 도포 대기")

            self.ref.update({"stt_mic_state": "tts_speaking"})
            tts_helper.speak("시멘트를 발라주세요", firebase_ref=self.ref, logger=self.get_logger())
            time.sleep(0.5)

            _wait_for_keyword(self._stt, CEMENT_KEYWORDS, "CEMENT/DONE", self.ref, self.get_logger())

            self.ref.update({"stt_mic_state": "tts_speaking"})
            tts_helper.speak("타일을 두세요", firebase_ref=self.ref, logger=self.get_logger())
            time.sleep(0.5)

            msg = Bool()
            msg.data = True
            self._publish_reliable(self._pub_cement_done, msg, retries=3)
            self.get_logger().info("[CEMENT] cement_done 토픽 전송 완료")

            tts_helper.speak("작업을 재개할게요", firebase_ref=self.ref, logger=self.get_logger())
            self.ref.update({
                "state": "시멘트 완료 - 작업 재개",
                "cement_state": "done",
                "stt_mic_state": "",
            })
            self.get_logger().info("[CEMENT] 재개 완료")

        except Exception as e:
            self.get_logger().error(f"[CEMENT] _cement_done_flow 오류: {e}")
        finally:
            self._cement_waiting = False

    def _handle_stt_trigger(self):
        """
        음성 녹음(STT) → 키워드 추출 → Firebase robot_status/stt_layout 저장.
        index.html이 stt_layout을 폴링하여 결과를 표시.
        """
        self.get_logger().info("[STT] 음성 녹음 시작")
        self.ref.update({"stt_layout": None, "stt_state": "recording"})

        try:
            if self._stt is None or self._keyword_extractor is None:
                raise RuntimeError("STT 또는 KeywordExtractor가 초기화되지 않았습니다.")

            text = self._stt.speech2text()
            self.get_logger().info(f"[STT] 인식 결과: {text}")
            self.ref.update({"stt_state": "extracting", "stt_text": text})

            layout = self._keyword_extractor.extract_keyword(text)
            if layout is None:
                raise ValueError(f"키워드 추출 실패: '{text}'")

            self.get_logger().info(f"[STT] layout: {layout}")
            self.ref.update({
                "stt_layout": layout,
                "stt_state": "done",
            })

        except Exception as e:
            self.get_logger().error(f"[STT] 오류: {e}")
            self.ref.update({"stt_state": "error", "stt_error": str(e)})