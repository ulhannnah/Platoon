"""
camera.py
카메라 캡처 지점을 하나로 모아둔 곳 (Jetson 기준).

라인트레이싱과 (나중에 붙일) 표지판 인식이 같은 프레임을 나눠 써야 하는데,
각자 카메라를 따로 열면 하드웨어 하나를 두 인스턴스가 붙잡으려다 충돌합니다.
그래서 캡처는 여기서만 하고, 인식 함수들은 프레임을 "받기만" 합니다.

라즈베리파이의 Picamera2와 달리, 젯슨은 카메라 종류에 따라 여는 방식이 다릅니다:
  - CSI 카메라(리본케이블, 예: IMX219) → GStreamer의 nvarguscamerasrc 파이프라인 필요
  - USB 웹캠 → OpenCV VideoCapture로 바로 열림 (V4L2)

어떤 카메라인지 코드에서 미리 알 수 없어서, USB로 먼저 시도하고 실패하면 CSI로
넘어가는 순서로 시도합니다. 실제 카메라 종류를 알면 camera_type 인자로 못박아도 됩니다.
"""

import cv2
import numpy as np


def _csi_gstreamer_pipeline(
    sensor_id=0, width=640, height=480, framerate=30, flip_method=0
):
    """젯슨 CSI 카메라(nvarguscamerasrc)용 GStreamer 파이프라인 문자열"""
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width={width}, height={height}, "
        f"format=NV12, framerate={framerate}/1 ! "
        f"nvvidconv flip-method={flip_method} ! "
        f"video/x-raw, width={width}, height={height}, format=BGRx ! "
        f"videoconvert ! video/x-raw, format=BGR ! appsink drop=1"
    )


class CameraCapture:
    """
    camera_type: "usb" / "csi" / "auto"(기본, USB 먼저 시도 후 CSI)
    device_index: USB 웹캠일 때 /dev/video 번호
    """

    def __init__(self, size=(640, 480), camera_type="auto", device_index=0):
        self.size = size
        self.cap = None
        self.backend = None

        order = ["usb", "csi"] if camera_type == "auto" else [camera_type]
        errors = []
        for kind in order:
            cap = None
            try:
                if kind == "usb":
                    cap = cv2.VideoCapture(device_index)
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, size[0])
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])
                else:
                    pipeline = _csi_gstreamer_pipeline(width=size[0], height=size[1])
                    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

                if not cap.isOpened():
                    raise RuntimeError(f"{kind} 카메라 열기 실패 (isOpened=False)")

                ok, _ = cap.read()
                if not ok:
                    raise RuntimeError(f"{kind} 카메라에서 프레임을 못 읽음")

                self.cap = cap
                self.backend = kind
                print(f"[camera] {kind} 카메라로 연결됨")
                return
            except Exception as e:
                errors.append(f"{kind}: {e}")
                if cap is not None:
                    cap.release()

        raise RuntimeError("카메라를 열 수 없습니다 — " + " / ".join(errors))

    def read(self) -> np.ndarray:
        """BGR 프레임 1장을 반환한다."""
        ok, frame = self.cap.read()
        if not ok:
            raise RuntimeError("프레임 읽기 실패 (카메라 연결이 끊겼을 수 있음)")
        return frame

    def close(self):
        if self.cap is not None:
            self.cap.release()
