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

## 3. 빌드

```bash
cd ~/platoon_ws
colcon build --packages-select platoon_control
source install/setup.bash
```

`source` 줄은 새 터미널을 열 때마다 다시 실행해야 합니다. 매번 치기 귀찮으면
`~/.bashrc`에 추가하세요:

```bash
echo "source ~/platoon_ws/install/setup.bash" >> ~/.bashrc
```

## 4. 실행

카메라·키보드 접근 때문에 보통 `sudo`가 필요합니다.

```bash
# 자동 주행으로 시작
ros2 run platoon_control platoon_control_node

# 수동(WASD)으로 시작
ros2 run platoon_control platoon_control_node --ros-args -p mode:=manual

# 런치 파일로 실행 (둘 다 동일하게 가능)
ros2 launch platoon_control platoon_control_launch.py mode:=manual
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

`platoon_ws/src/platoon_control/platoon_control/` 안의 파일들이 **실제로 실행되는
원본**입니다. `/mnt/user-data/outputs/`에 있던 원래 파일(`platoon_fsm.py` 등)을
고쳤다면, 이 안의 사본에도 **똑같이 반영해야** 실행에 반영됩니다. 지금은 둘이
따로 노는 복사본이라 자동 동기화되지 않습니다.

나중에 팀 전체가 ROS2로 넘어가면, 원본은 이 패키지 폴더 하나로 통일하고
`/mnt/user-data/outputs/`의 낱개 파일들은 정리하는 게 좋습니다.

## 7. 지금 이 패키지의 구조 — 노드 1개

```
[platoon_control_node]  (프로세스 1개)
  ├─ PerceptionTracker  (카메라 캡처 — 내부 스레드, 아직 토픽 아님)
  ├─ PlatoonFSM          (판단)
  ├─ driving_control      (조향/속도 계산)
  └─ STM32Interface      (시리얼 직접 송수신)

퍼블리시하는 토픽: /platoon_control/status (String, 상태 모니터링용)
```

지금은 라인트레이싱도 이 노드 "안에서" 스레드로 돕니다. 나중에 라인트레이싱이
정말 별도 프로세스/노드로 분리되면, `perception.get_lane()` 호출 한 줄만
`/lane_tracing` 토픽 구독으로 바꾸면 됩니다 — FSM/제어 로직은 그대로입니다.

## 8. 자주 겪을 문제

| 증상 | 원인 후보 |
|---|---|
| `ros2: command not found` | ROS2 환경을 source 안 함 (`source /opt/ros/<distro>/setup.bash`) |
| 카메라 못 엶 | CSI/USB 종류 확인, `camera.py`의 `camera_type` 강제 지정 |
| 키보드 리스너 실패 | `sudo`로 실행했는지, 모니터+키보드가 보드에 직결됐는지 확인 |
| STM32 응답 없음 | `stm32_protocol.md` 바이너리 프로토콜로 STM32 쪽 교체 여부 확인 |
| `colcon: command not found` | ROS2 설치에 colcon이 빠졌을 수 있음 (`sudo apt install python3-colcon-common-extensions`) |
