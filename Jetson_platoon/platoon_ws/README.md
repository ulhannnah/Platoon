# platoon_control ROS2 패키지 — 빌드/실행 가이드

젯슨에 ROS2(Humble 등)가 이미 설치되어 있다는 가정입니다. 설치 자체는 다루지 않습니다.

## 1. 워크스페이스 배치

`platoon_ws` 폴더를 젯슨의 아무 위치에나 두면 됩니다. 예:

```bash
cp -r platoon_ws ~/platoon_ws
cd ~/platoon_ws
```

## 2. 의존 패키지 설치

```bash
pip3 install opencv-python numpy pyserial pynput --break-system-packages
```

opencv-python은 젯슨에 JetPack 기본 OpenCV가 이미 있을 수 있습니다. 이미 있으면
이 pip 설치는 건너뛰어도 됩니다 (오히려 충돌하면 시스템 OpenCV를 우선하세요).

라즈베리파이(팔로워) 차량이고 공식 카메라 모듈을 쓴다면 `picamera2`가 추가로
필요합니다 — 이건 `pip install`이 아니라 **apt로만** 설치됩니다.

```bash
sudo apt install -y python3-picamera2
```

젯슨에는 원래 없는 패키지라 (import 실패 시 자동으로 usb/csi 백엔드로 폴백),
젯슨 쪽에서는 이 단계를 건너뛰어도 됩니다.

## 3. 빌드

**`platoon_interfaces`(커스텀 메시지)를 `platoon_control`보다 먼저 빌드해야 합니다.**
`platoon_control`이 그 메시지 타입(`V2xPacket` 등)을 참조하기 때문입니다.

```bash
cd ~/platoon_ws
colcon build --packages-select platoon_interfaces
source install/setup.bash
colcon build --packages-select platoon_control
source install/setup.bash
```

순서를 바꿔서 `platoon_control`을 먼저 빌드하면 `ModuleNotFoundError: No module
named 'platoon_interfaces'`가 납니다. 매번 이 순서를 지키기 귀찮으면 그냥
`colcon build`(패키지 선택 없이 전체 빌드)를 쓰세요 — colcon이 의존성 순서를
알아서 맞춰줍니다.

`source` 줄은 새 터미널을 열 때마다 다시 실행해야 합니다. 매번 치기 귀찮으면
`~/.bashrc`에 추가하세요:

```bash
echo "source ~/platoon_ws/install/setup.bash" >> ~/.bashrc
```

## 4. 실행

카메라·키보드 접근 때문에 보통 `sudo`가 필요합니다.

**`ros2 run`은 노드 하나만 띄웁니다.** 노드 3개(v2x_node, lane_detector_node,
platoon_control_node)를 다 띄우려면 아래 보드별 런치 파일을 쓰세요 — 개별 노드
실행은 디버깅할 때만 쓰는 걸 권합니다.

```bash
# 젯슨(리더)
ros2 launch platoon_control jetson_launch.py

# 라즈베리파이(팔로워) — 캘리브레이션 파일은 차량마다 다르므로 보통 지정해서 실행
ros2 launch platoon_control rpi_launch.py calibration_file:=/home/pi/camera_params.npz
```

`jetson_launch.py`/`rpi_launch.py`는 `is_leader`/`camera_type` 기본값만 보드에
맞게 미리 채워둔 얇은 래퍼입니다 — 실제 노드 구성은 `platoon_control_launch.py`
하나뿐이라, 토픽·노드를 바꿔야 하면 그 파일만 고치면 양쪽에 다 반영됩니다.
직접 `platoon_control_launch.py`를 써도 되지만 그러면 `is_leader:=true`처럼
매번 다 명시해야 합니다.

개별 노드만 띄우고 싶을 때 (디버깅용):

```bash
# 자동 주행으로 시작 (v2x_node는 별도로 안 뜸)
ros2 run platoon_control platoon_control_node

# 수동(WASD)으로 시작
ros2 run platoon_control platoon_control_node --ros-args -p mode:=manual
```

실행 중에는 이전과 동일하게 **Q로 자동↔수동 전환**, **W/A/S/D로 조종**,
**SPACE로 브레이크**가 됩니다.

## 5. 상태 확인 (다른 터미널에서)

```bash
ros2 topic echo /platoon_control/status
```

지금 FSM이 무슨 상태인지, 조향/속도가 얼마로 나가는지 실시간으로 볼 수 있습니다.
관제/모니터링 화면을 만들 때 이 토픽을 구독하면 됩니다.

## 6. 코드를 고칠 때 주의할 점

`platoon_ws/src/` 안의 파일들이 **실제로 실행되는 유일한 원본**입니다. 다른
곳에 사본을 만들지 마세요 (`Claude.md` 규칙 1).

## 7. 지금 이 워크스페이스의 구조 — 노드 3개

```
[v2x_node]              [lane_detector_node]           [platoon_control_node]
ESP32(V2X) 통신           카메라+차선인식 담당              판단(FSM) + 조향·속도 계산
(아직 스텁)                                                + STM32 송수신
  │                          │                              │
  ├─/v2x/esp32_data──────────┼─────────────────────────>  구독 (주변 차량 목록)
  <──/v2x/outgoing───────────┼───────────────────────────  발행 (결합 요청 등 패킷)
  ──/v2x/incoming────────────┼─────────────────────────>  구독 (수신 패킷)
                             ──/lane_info─────────────>  구독 (차선 오프셋·분기 등)

              platoon_control_node 내부
              ├─ PlatoonFSM          (판단)
              ├─ driving_control      (조향/속도 계산)
              ├─ STM32Interface      (시리얼 직접 송수신)
              └─ ManualController    (WASD 수동조종 — 카메라와 무관, 노드 분리와 상관없이 그대로)

퍼블리시하는 토픽: /platoon_control/status (String, 상태 모니터링용)
```

V2X 토픽과 `/lane_info`는 전부 `platoon_interfaces` 패키지의 타입 있는
메시지(`V2xPacket`, `Esp32Data`, `LaneInfo`)를 씁니다 — `ros2 topic echo`로
필드가 그대로 보입니다.

`lane_detector_node`는 카메라를 열고(`camera.py`) 차선을 인식해서(`lane_tracing.py`)
`/lane_info`로 발행합니다. `LaneInfo.offset`은 **픽셀 정수**라서,
`platoon_control_node`가 구독할 때 `lane_image_width` 파라미터(기본 640, 반드시
`lane_detector_node`의 캡처 폭과 일치해야 함)로 정규화(-1~1)합니다.

## 8. 자주 겪을 문제

| 증상 | 원인 후보 |
|---|---|
| `ros2: command not found` | ROS2 환경을 source 안 함 (`source /opt/ros/<distro>/setup.bash`) |
| `ModuleNotFoundError: platoon_interfaces` | `platoon_interfaces`를 먼저 빌드 안 함 (§3 순서 확인) |
| 카메라 못 엶 | 백엔드 자동감지 실패 — `camera_type` 파라미터로 `picamera2`/`usb`/`csi` 강제 지정 |
| picamera2 못 씀 | `pip install`이 아니라 `sudo apt install python3-picamera2`로 설치해야 함 |
| 키보드 리스너 실패 | `sudo`로 실행했는지, 모니터+키보드가 보드에 직결됐는지 확인 |
| STM32 응답 없음 | `stm32_protocol.md` 바이너리 프로토콜로 STM32 쪽 교체 여부 확인 |
| `colcon: command not found` | ROS2 설치에 colcon이 빠졌을 수 있음 (`sudo apt install python3-colcon-common-extensions`) |
| `v2x_node`가 `ros2 node list`에 안 보임 | `ros2 launch`가 아니라 `ros2 run platoon_control platoon_control_node`로 노드 하나만 띄운 경우 — §4처럼 런치 파일 써야 둘 다 뜸 |
