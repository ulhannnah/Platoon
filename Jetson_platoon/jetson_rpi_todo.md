# Jetson/RPi5 측 필요 작업 정리 (STM32 코드 기준)

STM32 레포(`Platoon_STM32`)의 실제 구현 상태와 Jetson 레포(`Platoon-Platoon1/Jetson_platoon`)의
현재 코드(`main.py`, `driving_control.py`, `platoon_fsm.py`, `stm32_interface.py`)를 맞대본 결과입니다.
STM32 쪽 프로토콜/엔코더/PID 재작업이 끝나는 것과 별개로, **Jetson 쪽에서도 손볼 부분**이 있습니다.

---

## 1. 코드 갭 — 값은 오는데 안 쓰고 있음

### 1.1 초음파 거리(`front_distance`) 값을 전혀 사용하지 않음

`stm32_interface.py`의 `DriveFeedback.front_distance`는 STM32가 올려주는 초음파 실측 거리(m)를
담는 필드입니다. STM32 쪽 코드(`supersonic.c`, `PB6 ECHO` + `TIM4` 입력캡처)를 보면 초음파 센서는
**STM32에 직결**되어 있고, `stm32_protocol.md`에도 "PLATOON_MAINTAIN 단계의 주 거리 센서는 초음파
(UWB는 보조)"라고 명시돼 있습니다.

그런데 지금 `main.py`는 `feedback.front_distance`를 **전혀 읽지 않고**, `feedback.obstacle`
(불리언 플래그)만 씁니다. 거리 제어(`compute_control`에 넘기는 `uwb_distance`)는 전부 ESP32의
UWB 값(`partner.uwb_distance`)만 사용 중입니다 (`platoon_fsm.py`, `driving_control.py`).

→ **팀 결정 필요**: 설계문서대로 초음파를 MAINTAIN 단계의 주 거리값으로 실제 제어 루프에
반영할지, 아니면 지금처럼 UWB를 주로 쓰고 초음파는 근접 안전정지(안전용 backup)로만 남길지
정해야 합니다. 반영하기로 하면 `main.py`에서 `feedback.front_distance`를 `compute_control()`
호출부로 연결하는 코드가 필요합니다.

### 1.2 `feedback.failsafe` 를 아예 안 읽음

STM32가 RPi 무응답(200ms 초과)을 감지하면 `STATUS_FAILSAFE` 비트를 세워 보내주는데,
`main.py`는 `feedback.obstacle`만 읽고 `feedback.failsafe`는 코드 어디에서도 참조하지 않습니다.

STM32는 자체적으로 정지하지만, Jetson의 FSM/제어 로직은 이 사실을 모른 채 계속 목표 속도를
계산해서 내려보낼 수 있습니다(차가 멈췄는데 FSM은 "주행 중"으로 착각). 최소한 로그를 남기거나,
`failsafe`가 True인 동안은 FSM을 안전 상태로 전이시키는 처리를 추가하는 게 좋습니다.

### 1.3 `EgoState.obstacle` 필드가 선언만 되고 로직에서 안 쓰임

`platoon_fsm.py`에 `obstacle: bool = False` 필드는 있지만, FSM의 어떤 판단 로직도 이 값을
참조하지 않습니다(감속, 정지, 상태 전이 등 어디에도 안 걸림). `main.py`에서 값은 채워주고
있으니(`ego.obstacle = feedback.obstacle`), FSM 쪽에 "장애물 감지 시 어떻게 반응할지"
로직을 추가해야 실제로 동작합니다.

---

## 2. STM32와 반드시 값을 맞춰야 하는 상수

| 파라미터 | Jetson 현재값 | STM32 실제값/제약 | 확인할 것 |
|---|---|---|---|
| `MAX_STEERING_RAD` (`driving_control.py`) | 30° (임시) | 서보 하드웨어 범위 -45°~+45° (`servo.h`, 1000~2000µs) | 30°가 의도된 마진인지, 45°로 맞출지 팀 결정. 어느 쪽이든 STM32 서보 설정과 **동일 값**이어야 함 |
| `WHEELBASE_M` (`driving_control.py`) | 0.20 (임시) | — (기구 설계값) | 줄자 실측 필요. pure pursuit 조향각 전체에 비례 영향 |
| 시리얼 포트 (`main.py` `open_stm32_port()`) | `/dev/ttyAMA0` 하드코딩 | STM32는 `USART2` 고정 | RPi5 실제 배선(GPIO UART vs USB-시리얼 어댑터) 확인 후 포트 경로 확정 |
| Baudrate | 115200 | 115200 (`serial_communication.c` 기준) | 현재는 일치. 패킷 유실 잦으면 460800으로 **양쪽 동시에** 올려야 함 |
| 조향 부호 | 좌회전 = 양수 (변경 없음) | STM32에서 뒤집기로 합의됨 (`stm32_protocol.md` §5.2) | Jetson 쪽 코드 수정은 불필요. 실차에서 좌/우 반대로 도는지 **검증만** 필요 |

---

## 3. STM32가 아직 완성 전이라 생기는 임시 이슈

STM32 쪽은 현재 구프로토콜(ASCII, `PWM:...`/`M:...`)을 쓰고 있고, `stm32_protocol.md`가 정의한
바이너리 프로토콜 + 엔코더/PID 폐루프 제어는 아직 미구현 상태입니다 (별도로 정리한 STM32 작업
목록 참고). 이게 끝나기 전까지 Jetson 쪽에서 알아둘 점:

- **`stm32_interface.py`는 이미 바이너리 프로토콜로 완성돼 있어서**, STM32가 프로토콜을 갈아엎기
  전까지는 실제 보드 간 통신 테스트 자체가 안 됩니다. Jetson 쪽은 추가 구현 없이 대기하면 되고,
  STM32 교체 완료 후 통합 테스트만 진행하면 됩니다.
- **`current_speed` 피드백 신뢰 불가**: STM32에 실제 엔코더 입력이 없어서(현재 `encoder.c`는
  개루프 PWM 제어만 함) `current_speed_mmps`를 채울 방법이 없습니다. `main.py`는 이미
  `feedback is None`이면 이전 속도값을 유지하는 fallback이 있는데, 엔코더가 붙기 전까지는
  이 fallback 경로가 계속 타게 됩니다 — 통합 테스트 때 이 부분이 정상 동작인지 헷갈리지 않도록
  인지하고 있어야 합니다.

---

## 4. 팀에서 "결정"만 하면 되는 항목 (측정 대상 아님)

`paramter.md` §7에 정리된 것 중 코드/문서에 직접 걸리는 것들:

- `vehicle_id` — `main.py`의 하드코딩된 `1`을 차량별 고유 ID 부여 방식으로 교체
- `reason_code` 체계 — `v2x_protocol.py`의 `PlatoonJoinReject` 거절 사유 코드 정의
- `CHECKPOINT_ADJACENT_RANGE`, `lane_ok`(현재 항상 True) — 체크포인트/트랙 차선 구조 확정 후 값 결정
- `LANE_ALIGN_LATERAL_TOLERANCE_M` (0.15m 기준값) — 차폭/차선폭 고려해 확정

---

## 5. 통합 테스트 체크리스트 (STM32 업데이트 완료 후)

1. 루프백 — STM32가 보낸 피드백을 `stm32_interface.py`의 `PacketParser`가 정상 파싱하는지
2. 체크섬 오류 주입 — 일부러 깨진 프레임을 보내고 양쪽 다 조용히 폐기하는지
3. 케이블 중간 절단/재연결 — 헤더 재탐색으로 정상 복구되는지
4. Failsafe — Jetson 쪽 송신을 멈추고 200ms 후 STM32가 실제로 멈추는지, 그리고 이때
   `feedback.failsafe`를 Jetson이 인지하는지 (§1.2 구현 후)
5. 초음파 값 반영 여부 결정 후 — MAINTAIN 상태에서 초음파/UWB 중 실제로 쓰기로 한 값이
   거리 제어에 제대로 반영되는지
