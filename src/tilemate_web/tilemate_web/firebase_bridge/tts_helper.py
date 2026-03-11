"""
firebase_bridge/tts_helper.py
TTS(gTTS + pygame) 재생 및 Firebase 업로드 헬퍼
"""

import base64
import os
import tempfile
import time

try:
    from gtts import gTTS
    import pygame
    _TTS_AVAILABLE = True
except ImportError:
    _TTS_AVAILABLE = False
    print("[WARNING] gTTS 또는 pygame 없음. TTS 비활성화. pip install gtts pygame")


def init_pygame_mixer(logger=None):
    """pygame.mixer 초기화. TTS 재생 전 한 번만 호출."""
    if not _TTS_AVAILABLE:
        return
    try:
        pygame.mixer.init()
    except Exception as e:
        if logger:
            logger.warn(f"[TTS] pygame.mixer 초기화 실패: {e}")


def speak(text: str, firebase_ref=None, logger=None):
    """
    텍스트를 한국어 TTS로 재생.
    gTTS로 mp3 생성 → pygame 재생.
    firebase_ref가 있으면 base64로 인코딩해 Firebase에도 저장.
    """
    if not _TTS_AVAILABLE:
        if logger:
            logger.warn(f"[TTS] 비활성화 상태. 출력 생략: '{text}'")
        return
    try:
        with tempfile.NamedTemporaryFile(suffix=".mp3", delete=False) as f:
            tmp_path = f.name
        tts = gTTS(text=text, lang="ko")
        tts.save(tmp_path)

        if firebase_ref:
            try:
                with open(tmp_path, "rb") as f:
                    b64 = base64.b64encode(f.read()).decode("utf-8")
                firebase_ref.update({"tts_audio_b64": b64, "tts_audio_text": text})
            except Exception as e:
                if logger:
                    logger.warn(f"[TTS] Firebase 업로드 실패: {e}")

        pygame.mixer.music.load(tmp_path)
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():
            time.sleep(0.1)

        os.remove(tmp_path)
        if logger:
            logger.info(f"[TTS] 재생 완료: '{text}'")
    except Exception as e:
        if logger:
            logger.error(f"[TTS] 오류: {e}")


def upload_tts_preset(text: str, firebase_key: str, firebase_ref=None, logger=None):
    """
    gTTS로 mp3 생성 후 base64로 Firebase에 사전 저장.
    웹에서 동일한 목소리로 재생하기 위한 프리셋 업로드.
    """
    if not _TTS_AVAILABLE or firebase_ref is None:
        return
    try:
        with tempfile.NamedTemporaryFile(suffix=".mp3", delete=False) as f:
            tmp_path = f.name
        tts = gTTS(text=text, lang="ko")
        tts.save(tmp_path)

        with open(tmp_path, "rb") as f:
            b64 = base64.b64encode(f.read()).decode("utf-8")
        os.remove(tmp_path)
        firebase_ref.update({firebase_key: b64})
        if logger:
            logger.info(f"[TTS] preset 업로드 완료: '{text}' → {firebase_key}")
    except Exception as e:
        if logger:
            logger.warn(f"[TTS] preset 업로드 실패: {e}")
