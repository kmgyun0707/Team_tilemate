from openai import OpenAI
import sounddevice as sd
import numpy as np
import tempfile
import os
import wave

from dotenv import load_dotenv

load_dotenv(dotenv_path=os.path.join(".env"))
openai_api_key = os.getenv("OPENAI_API_KEY")


class STT:
    def __init__(self, openai_api_key):
        self.client = OpenAI(api_key=openai_api_key)
        self.samplerate = 16000          # Whisper는 16kHz 선호
        self.chunk_duration = 0.05       # 50ms 단위로 오디오 처리
        self.chunk_size = int(self.samplerate * self.chunk_duration)

        # VAD 파라미터
        self.silence_threshold = None    # None이면 시작 시 자동 캘리브레이션
        self.calibration_duration = 1.0  # 배경 소음 측정 시간 (초)
        self.calibration_multiplier = 1.5  # 배경소음 RMS × 배수 = threshold
        self.min_threshold = 40          # threshold 최솟값 (너무 민감해지지 않도록)
        self.min_speech_duration = 0.15   # 최소 발화 길이 (초) — 노이즈 필터
        self.silence_timeout = 1.5       # 발화 종료 판단 무음 시간 (초)
        self.max_duration = 30           # 최대 녹음 시간 (초)

        # 캘리브레이션 완료 후 청취 시작 시점 콜백 (선택)
        # 예: self._stt.on_listening = lambda: ref.update({"stt_mic_state": "listening"})
        self.on_listening = None

    def _calibrate(self) -> float:
        """배경 소음 RMS를 측정해 threshold 자동 설정"""
        print(f"배경 소음 측정 중... ({self.calibration_duration}초)")
        n_chunks = int(self.calibration_duration / self.chunk_duration)
        rms_values = []

        with sd.InputStream(
            samplerate=self.samplerate,
            channels=1,
            dtype="int16",
            blocksize=self.chunk_size,
        ) as stream:
            for _ in range(n_chunks):
                chunk, _ = stream.read(self.chunk_size)
                rms_values.append(self._rms(chunk.flatten()))

        bg_rms = float(np.mean(rms_values))
        threshold = max(self.min_threshold, bg_rms * self.calibration_multiplier)
        print(f"배경 소음 RMS={bg_rms:.1f} → threshold={threshold:.1f} 으로 설정")
        return threshold

    def _rms(self, audio_chunk: np.ndarray) -> float:
        """오디오 청크의 RMS(에너지) 계산"""
        return float(np.sqrt(np.mean(audio_chunk.astype(np.float32) ** 2)))

    def _save_wav(self, audio_data: np.ndarray, filepath: str):
        """numpy 배열을 WAV 파일로 저장"""
        with wave.open(filepath, 'w') as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)  # int16 = 2 bytes
            wf.setframerate(self.samplerate)
            wf.writeframes(audio_data.tobytes())

    def speech2text(self) -> str:
        """
        VAD 기반 음성 인식:
        - 말이 시작될 때 자동으로 녹음 시작
        - 말이 끝나면 (무음 지속) 자동으로 녹음 종료
        - Whisper API로 텍스트 변환
        """
        # threshold가 없을 때만 캘리브레이션 (재시도 시엔 기존 값 유지)
        if self.silence_threshold is None:
            self.silence_threshold = self._calibrate()
            # 캘리브레이션 완료 → 청취 시작 알림
            if callable(self.on_listening):
                try:
                    self.on_listening()
                except Exception:
                    pass
        else:
            # 재시도: 캘리브레이션 없이 바로 청취 시작 알림
            if callable(self.on_listening):
                try:
                    self.on_listening()
                except Exception:
                    pass

        print(f"대기 중... (말씀하시면 자동으로 녹음됩니다) [threshold={self.silence_threshold:.1f}]")

        recorded_chunks = []
        silence_chunks = 0
        speech_chunks = 0
        is_speaking = False

        silence_limit = int(self.silence_timeout / self.chunk_duration)
        min_speech_chunks = int(self.min_speech_duration / self.chunk_duration)
        max_chunks = int(self.max_duration / self.chunk_duration)
        total_chunks = 0

        with sd.InputStream(
            samplerate=self.samplerate,
            channels=1,
            dtype="int16",
            blocksize=self.chunk_size,
        ) as stream:
            while total_chunks < max_chunks:
                chunk, _ = stream.read(self.chunk_size)
                chunk = chunk.flatten()
                rms = self._rms(chunk)
                total_chunks += 1

                if rms > self.silence_threshold:
                    # 발화 감지
                    if not is_speaking:
                        print("녹음 시작...")
                        is_speaking = True
                    recorded_chunks.append(chunk)
                    speech_chunks += 1
                    silence_chunks = 0

                elif is_speaking:
                    # 발화 중 무음 구간 — 버퍼에 포함 (자연스러운 끊김 처리)
                    recorded_chunks.append(chunk)
                    silence_chunks += 1

                    if silence_chunks >= silence_limit:
                        # 무음이 충분히 지속 → 발화 종료
                        print("발화 종료 감지.")
                        break

                # 발화 시작 전 무음은 버리기 (대기 상태)

        if not recorded_chunks or speech_chunks < min_speech_chunks:
            print(f"발화가 감지되지 않았습니다. (threshold={self.silence_threshold:.1f} — 너무 높으면 self.silence_threshold 값을 낮춰보세요)")
            return ""

        audio_data = np.concatenate(recorded_chunks, axis=0)
        print(f"녹음 완료 ({len(audio_data) / self.samplerate:.1f}초). Whisper에 전송 중...")

        # 임시 WAV 파일로 저장 후 Whisper API 호출
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
            tmp_path = tmp.name

        try:
            self._save_wav(audio_data, tmp_path)
            with open(tmp_path, "rb") as f:
                transcript = self.client.audio.transcriptions.create(
                    model="whisper-1",
                    file=f,
                    language="ko",
                )
        finally:
            os.remove(tmp_path)

        print("STT 결과:", transcript.text)
        return transcript.text


if __name__ == "__main__":
    stt = STT(openai_api_key)

    # 연속 인식 루프 (Ctrl+C로 종료)
    while True:
        try:
            result = stt.speech2text()
            if result:
                print(f"[인식됨] {result}\n")
        except KeyboardInterrupt:
            print("\n종료합니다.")
            break