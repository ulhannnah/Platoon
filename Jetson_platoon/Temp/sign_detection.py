"""
sign_detection.py
표지판 인식 (체크포인트 판별용) — 아직 모델 학습 전이라 스텁입니다.

lane_tracing.py의 PerceptionTracker가 라인트레이싱과 같은 카메라 프레임을
이 모듈에도 넘겨줍니다. 카메라를 새로 열 필요 없이, detect_signs(frame) 함수
안의 TODO만 실제 추론 코드로 채우면 자동으로 연결됩니다.
"""

from dataclasses import dataclass
from typing import Optional


@dataclass
class SignDetectionResult:
    sign_id: Optional[int] = None    # 인식된 표지판 종류 — 체크포인트 번호와 매핑
    confidence: float = 0.0


def detect_signs(frame) -> Optional[SignDetectionResult]:
    """
    TODO: 표지판 학습 완료 후 실제 추론 코드(TensorRT 등)로 교체.
    지금은 항상 None을 반환해서 PerceptionTracker가 이 결과를 무시하게 한다.
    """
    return None