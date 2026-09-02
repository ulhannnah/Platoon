"""
camera.py
카메라 캡처 지점을 하나로 모아둔 곳.

라인트레이싱과 (나중에 붙일) 표지판 인식이 같은 프레임을 나눠 써야 하는데,
각자 카메라를 따로 열면 하드웨어 하나를 두 인스턴스가 붙잡으려다 충돌합니다.
그래서 캡처는 여기서만 하고, 인식 함수들은 프레임을 "받기만" 합니다.

차량 보드에 따라 카메라 여는 방식이 다릅니다:
  - 라즈베리파이 + 공식 카메라 모듈  → picamera2 (libcamera 기반)
  - 젯슨 + CSI 카메라(리본케이블, 예: IMX219) → GStreamer의 nvarguscamerasrc 파이프라인
  - USB 웹캠(양쪽 다)              → OpenCV VideoCapture로 바로 열림 (V4L2)

어떤 조합인지 코드에서 미리 알 수 없어서, picamera2 → USB → CSI(GStreamer) 순으로
시도합니다. 실제 카메라 종류를 알면 camera_type 인자로 못박아도 됩니다.

picamera2/젯슨 CSI 경로는 서로 다른 하드웨어 전제라, 한쪽이 없는 보드에서는
그냥 import/파이프라인 생성이 실패하고 다음 후보로 넘어갑니다 — 두 경로가
동시에 필요한 게 아니라 "이 차가 어느 쪽이든 알아서 맞는 걸 찾는" 구조입니다.
"""

import cv2
import numpy as np

try:
    from picamera2 import Picamera2
except ImportError:
    Picamera2 = None  # 젯슨 등 picamera2가 없는 환경 — USB/CSI로 폴백


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


class _Picamera2Wrapper:
    """cv2.VideoCapture와 같은 read()/isOpened()/release() 인터페이스로 Picamera2를 감싼다."""

    def __init__(self, size):
        self._cam = Picamera2()
        config = self._cam.create_preview_configuration(
            main={"size": size, "format": "RGB888"}
        )
        self._cam.configure(config)
        self._cam.start()

    def isOpened(self) -> bool:
        return True

    def read(self):
        # picamera2는 RGB로 주는데, 이 프로젝트의 나머지 코드(lane_tracing.py 등)는
        # 전부 cv2 기준 BGR을 전제로 하므로 여기서 한 번만 변환해둔다.
        frame = self._cam.capture_array("main")
        return True, cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    def release(self):
        self._cam.stop()
        self._cam.close()


class CameraCapture:
    """
    camera_type      : "picamera2" / "usb" / "csi" / "auto"(기본, picamera2→usb→csi 순)
    device_index     : USB 웹캠일 때 /dev/video 번호
    calibration_file : 렌즈 왜곡보정 npz 경로 ("mtx", "dist" 배열 포함).
                       None이면 보정 없이 원본 프레임을 그대로 반환.
                       npz 형식은 OpenCV cv2.calibrateCamera()의 표준 산출물과 동일.
    """

    def __init__(self, size=(640, 480), camera_type="auto", device_index=0,
                 calibration_file=None):
        self.size = size
        self.cap = None
        self.backend = None
        self._undistort_map = None

        order = (["picamera2", "usb", "csi"] if camera_type == "auto"
                 else [camera_type])
        errors = []
        for kind in order:
            cap = None
            try:
                if kind == "picamera2":
                    if Picamera2 is None:
                        raise RuntimeError("picamera2 모듈 없음 (라즈베리파이가 아니거나 미설치)")
                    cap = _Picamera2Wrapper(size)
                elif kind == "usb":
                    cap = cv2.VideoCapture(device_index)
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, size[0])
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])
                elif kind == "csi":
                    pipeline = _csi_gstreamer_pipeline(width=size[0], height=size[1])
                    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
                else:
                    raise ValueError(f"알 수 없는 camera_type: {kind}")

                if not cap.isOpened():
                    raise RuntimeError(f"{kind} 카메라 열기 실패 (isOpened=False)")

                ok, _ = cap.read()
                if not ok:
                    raise RuntimeError(f"{kind} 카메라에서 프레임을 못 읽음")

                self.cap = cap
                self.backend = kind
                print(f"[camera] {kind} 카메라로 연결됨")
                break
            except Exception as e:
                errors.append(f"{kind}: {e}")
                if cap is not None:
                    cap.release()
        else:
            raise RuntimeError("카메라를 열 수 없습니다 — " + " / ".join(errors))

        if calibration_file:
            self._load_calibration(calibration_file)

    def _load_calibration(self, path: str) -> None:
        try:
            data = np.load(path)
            self._undistort_map = cv2.initUndistortRectifyMap(
                data["mtx"], data["dist"], None, data["mtx"], self.size, cv2.CV_16SC2
            )
            print(f"[camera] 렌즈 왜곡보정 로드됨: {path}")
        except Exception as e:
            print(f"[warn] 캘리브레이션 파일 로드 실패({path}), 보정 없이 진행: {e}")
            self._undistort_map = None

    def read(self) -> np.ndarray:
        """BGR 프레임 1장을 반환한다 (캘리브레이션 파일이 있으면 왜곡보정까지 적용됨)."""
        ok, frame = self.cap.read()
        if not ok:
            raise RuntimeError("프레임 읽기 실패 (카메라 연결이 끊겼을 수 있음)")
        if self._undistort_map is not None:
            frame = cv2.remap(frame, *self._undistort_map, interpolation=cv2.INTER_LINEAR)
        return frame

    def close(self):
        if self.cap is not None:
            self.cap.release()
