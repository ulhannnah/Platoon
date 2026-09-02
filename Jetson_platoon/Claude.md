# Jetson_platoon 프로젝트 컨텍스트

한이음 드림업 2026 팀 프로젝트 "AI 기반 협력 자율주행 플래툰 제어 및 의사결정 시스템"
(팀명: 군집돌). 이 문서 작성자는 **플래툰 알고리즘(FSM, 적합도 판단, 통신 프로토콜)**
담당이며, STM32 모터제어와 라인트레이싱은 다른 팀원 담당.

## 하드웨어 아키텍처 (3계층)

```
[ESP32-S3 + UWB]  V2X 통신 + UWB 거리·각도 연산 (원시 신호 처리는 여기서 끝남)
        ↓ 이미 계산된 거리(m)·각도(rad)만 전달
[젯슨]            판단(FSM) + 조향/속도 계산  ← 이 레포가 담당하는 부분
        ↓ 목표 조향각(rad) / 목표 속도(m/s)만 전달
[STM32]           서보 PWM 변환, 엔코더 기반 모터 PID
```

경계 원칙: 젯슨 쪽 코드는 PWM·모터 PID·UWB 원시신호 처리를 절대 하지 않는다.
"얼만큼 갈지"까지만 계산해서 넘긴다.

## 실차 시나리오 (확정된 설계 결정)

- **리더 차량은 항상 젯슨 1대로 고정.** 합류/이탈/팔로워는 전부 라즈베리파이 차량들.
  역할이 UWB 각도 등으로 동적으로 정해지지 않고 보드 종류로 미리 고정된다
  (`PlatoonFSM(is_designated_leader=...)`).
- **조향(횡방향)은 리더든 팔로워든 항상 카메라 라인트레이싱을 쓴다.** "팔로워는
  V2V만으로 주행"이 아니다 — 라인트레이싱을 하면서 동시에 V2V로 군집주행한다.
  V2V/UWB는 속도·차간거리(종방향)만 담당. (예외: 플래툰 중 라인 유실 시에만
  UWB pure pursuit로 잠깐 버팀 — 뒤차가 붙어있어 급정지가 더 위험하므로)
- **3대 이상 체인 지원**: 리더-팔로워1-팔로워2-... 순으로 맨 뒤에만 결합.
  `leader_id`(전체 리더, feedforward용)와 `partner_id`(바로 앞차, 거리제어용)는
  별개 개념 — 혼동하면 3번째 차가 2번째 차를 리더로 착각하는 버그가 남.

## 레포 구조

```
platoon_ws/                          ← 유일한 원본. 수정은 여기서만 한다.
  src/platoon_interfaces/            ← 커스텀 ROS2 메시지 패키지 (ament_cmake).
                                         platoon_control보다 먼저 빌드해야 함
                                         (colcon build --packages-select platoon_interfaces 먼저,
                                         그다음 platoon_control).
    msg/
      V2xPacket.msg                  ← 결합 프로토콜 패킷 10종 통합 메시지.
                                         필드명이 platoon_fsm.py의 dataclass와 1:1 동일
                                         (msg_type으로 어느 패킷인지 구분, 나머지 필드는
                                         그 타입에서만 의미 있고 나머지는 기본값)
      NearbyVehicleMsg.msg           ← NearbyVehicle dataclass 대응
      Esp32Data.msg                  ← read_esp32_packet() 반환값(dict) 대응
      LaneInfo.msg                   ← lane_detector_node.py 발행. 준호님(팀원) 레포와
                                         메시지 설계 동일 (패키지 이름 충돌 방지 위해 이
                                         패키지로 통합). offset은 픽셀 정수, 부호는 이
                                         레포 규약("차선이 오른쪽에 있으면 +") 유지 —
                                         준호님 원본 decision_node.py와 부호 반대이니
                                         그쪽 코드에 이 토픽을 그대로 연결하지 말 것
      VehicleCmd.msg, Telemetry.msg, FsmCmd.msg  ← 준호님 레포에서 가져옴. 지금은
                                         미사용 (STM32 통신은 stm32_interface.py를
                                         노드 안에서 직접 호출하는 구조라서). 나중에
                                         STM32 통신을 별도 노드로 뺄 때 대비해 패키지만
                                         통합해둠
  src/platoon_control/
    platoon_control/
      platoon_fsm.py                 ← 판단 (§5~§21 적합도/상태머신/통신) +
                                         V2X 패킷 정의·LoopbackBus(시뮬용).
                                         표준 라이브러리만 import하는 독립 파일 —
                                         다른 ROS2 노드에서 이 파일 하나만 떼어다
                                         쓸 수 있어야 해서 v2x_protocol.py를 합침.
                                         패킷 클래스를 다른 곳에서 새로 만들지 말 것
                                         (이름이 같아도 다른 클래스가 되어
                                         isinstance() 비교가 깨짐).
      ros_v2x_comm.py                ← V2X 노드 ↔ FSM 노드 ROS2 토픽 어댑터.
                                         platoon_interfaces의 타입 메시지 ↔
                                         platoon_fsm.py의 dataclass 변환 담당
      v2x_node.py                    ← ESP32(V2X) 통신 전용 ROS2 노드
      lane_detector_node.py          ← 카메라+차선인식 전용 ROS2 노드. camera.py로
                                         카메라를 열고(젯슨 호환 위해 이 부분만 각색),
                                         인식 알고리즘(process_frame)은 준호님(팀원)
                                         레포에서 그대로 가져옴. /lane_info(LaneInfo)
                                         발행. Flask 디버그 웹스트림 포함(선택사항).
                                         lane_tracing.detect_lane()은 이제 이 노드가
                                         안 씀 — 대체용으로 남겨둔 상태, 참고용
      driving_control.py             ← 조향각 계산 (카메라 PID + UWB pure pursuit 폴백)
      stm32_interface.py             ← STM32 UART 송수신
      manual_control.py              ← WASD 수동조종 (STM32 직접 제어, 카메라와 무관 —
                                         노드 분리와 상관없이 계속 platoon_control_node
                                         안에서 모듈로 씀)
      camera.py                      ← 카메라 캡처 (picamera2/usb/csi 자동감지 +
                                         선택적 렌즈왜곡보정). lane_detector_node.py 전용
      lane_tracing.py                ← 차선 인식 순수함수(detect_lane). rclpy 없음
      sign_detection.py              ← 표지판 인식 스텁. lane_detector_node.py가 호출
      platoon_control_node.py        ← ROS2 노드 (진입점)
    launch/
      platoon_control_launch.py      ← 노드 3개(v2x_node, lane_detector_node,
                                         platoon_control_node) 정의. 유일하게
                                         Node(...)가 있는 곳 — 여기만 고치면
                                         젯슨·라즈베리파이 둘 다 반영됨
      jetson/jetson_launch.py        ← 위 파일을 include하고 is_leader:=true 등
                                         젯슨용 기본값만 미리 채운 얇은 래퍼
      rpi/rpi_launch.py              ← 위 파일을 include하고 is_leader:=false,
                                         camera_type:=picamera2 등 라즈베리파이용
                                         기본값만 미리 채운 얇은 래퍼
tools/                                ← ROS2 불필요, python3로 바로 실행
  sim_two_vehicles.py                 ← LoopbackBus로 2~3대 결합 시뮬레이션 (회귀 테스트용)
  manual_drive.py                     ← FSM 없이 배선만 테스트
docs/
  260714_platoon_algorithm_design.md  ← 원본 설계문서. 코드 주석의 "§19.5" 같은
                                         절 번호는 전부 이 문서를 가리킴
  parameters.md                       ← 실측/튜닝 필요한 파라미터 전체 목록
  stm32_protocol.md                   ← STM32 담당자용 UART 프로토콜 명세
  jetson_rpi_todo.md                  ← 인터페이스 갭 정리
  mentor_briefing.md                  ← 멘토 보고용 요약
  ros2_node_architecture.md           ← 노드/토픽 구조 설명 (팀원 온보딩용)
```

## 작업 규칙

1. **`platoon_ws/src/platoon_control/platoon_control/` 안 파일만 수정한다.**
   다른 곳에 사본을 만들지 않는다.
2. **파일을 zip으로 묶어서 주지 않는다.** 개별 파일로 제시.
3. 로직을 고치면 `tools/sim_two_vehicles.py`로 회귀 테스트 후 결과를 보여준다
   (통신 하드웨어 없이도 FSM 로직은 이걸로 검증 가능 — `LoopbackBus`가 ESP-NOW를
   메모리에서 흉내냄).
4. 코드 주석의 "§n" 표기는 `docs/260714_platoon_algorithm_design.md`의 절 번호다.
   새 로직을 짤 때 그 문서에 관련 절이 있는지 먼저 확인한다.
5. 실측 전 임시값(제어 이득, 임계거리 등)은 `docs/parameters.md`에 등재하고
   TODO 주석을 남긴다.

## 아직 안 된 것 (우선순위 순)

1. **ESP32 프로토콜** — `platoon_control_node.py`의 `read_esp32_packet()`이 아직
   빈 스텁. STM32 때처럼 문서(C 구조체 등) + `esp32_interface.py` 세트로 필요.
2. 체크포인트/경로 인식 — `EgoState.checkpoint`/`route`를 채울 방법 (표지판 인식
   CNN 예정, `sign_detection.py`가 자리만 잡아둔 상태)
3. 실측 파라미터 반영 (`docs/parameters.md` 체크리스트)
4. 강화학습(Parameter RL) — 제어 이득을 상황 적응적으로 튜닝. 지금은 시기상조,
   ESP32 붙고 실측값 채운 뒤 진행