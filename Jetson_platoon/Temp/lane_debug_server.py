"""
lane_debug_server.py
브라우저에서 라인트레이싱 처리 과정을 실시간으로 보는 디버그 뷰어.
원본 Flask 코드와 같은 화면이지만, 알고리즘은 lane_tracing.py의 detect_lane()을
그대로 재사용합니다 (로직 중복 없음 — 여기 고치면 저기도 같이 고쳐지는 게 아니라,
아예 같은 함수를 부릅니다).

하드웨어/파라미터가 아직 임시값이라 눈으로 튜닝할 때 씁니다.

실행:
    python3 lane_debug_server.py
    브라우저에서 http://<라즈베리파이IP>:5000 접속

주의: main.py와 동시에 켜면 카메라를 두 번 열게 되어 충돌합니다.
      main.py를 끄고 튜닝할 때만 켜세요.
"""

import cv2
from flask import Flask, Response

from lane_tracing import PerceptionTracker

app = Flask(__name__)
tracker = PerceptionTracker()

INDEX_HTML = """
<!DOCTYPE html>
<html>
<head><meta charset="UTF-8"><title>Lane Tracing Debug</title>
<style>body{margin:0;background:#000}img{width:100vw;height:100vh;object-fit:contain}</style>
</head>
<body><img src="/video_feed"></body>
</html>
"""


def generate_frames():
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 75]
    while True:
        frame = tracker.get_debug_frame()
        if frame is None:
            continue
        ok, buffer = cv2.imencode(".jpg", frame, encode_param)
        if not ok:
            continue
        yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + buffer.tobytes() + b"\r\n")


@app.route("/")
def index():
    return INDEX_HTML


@app.route("/video_feed")
def video_feed():
    return Response(generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")


if __name__ == "__main__":
    if not tracker.available:
        print("카메라를 못 열었습니다. 라즈베리파이에서 실행 중인지 확인하세요.")
    app.run(host="0.0.0.0", port=5000, threaded=True)