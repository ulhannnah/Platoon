# ESP32-S3 + DW3000 AoA 기반 플레툰 V2V 통신 구조 정리

## 1. 프로젝트 개요

본 프로젝트는 플레툰 자율주행 차량 간 통신 및 상대 위치 추정을 위한 시스템이다.  
각 차량에는 **ESP32-S3**와 **DW3000 기반 UWB AoA 보드**를 장착하고, 차량 간 상태 정보는 ESP-NOW 또는 Wi-Fi 계열 통신으로 공유하며, 거리와 방향 정보는 UWB AoA를 통해 측정한다.  
최종적으로 ESP32-S3는 수집한 정보를 유선 통신으로 Raspberry Pi에 전달하고, Raspberry Pi는 상대 위치 추정, 플레툰 제어, 로그 저장을 담당한다.

핵심 목표는 다음과 같다.

- 차량 간 고유 ID 공유
- UWB 기반 거리 측정
- AoA 기반 방위각 측정
- 차량 ID와 UWB ID 매칭
- 상대 차량의 2D 위치 추정
- Raspberry Pi로 유선 데이터 전달
- 플레툰 제어 알고리즘에 사용할 수 있는 안정적인 상대 위치 정보 생성

---

## 2. 전체 시스템 구조

```text
[차량 A]
 ┌────────────────────────┐
 │ Raspberry Pi            │
 │ - 자율주행 판단          │
 │ - 플레툰 제어            │
 │ - 상대좌표 추정          │
 │ - 로그 저장              │
 └──────────▲─────────────┘
            │ UART / USB / Serial
 ┌──────────┴─────────────┐
 │ ESP32-S3                │
 │ - ESP-NOW 통신           │
 │ - UWB 제어               │
 │ - ID 매칭                │
 │ - TDMA 슬롯 관리          │
 │ - 데이터 패킷화           │
 └──────▲──────────▲──────┘
        │ SPI      │ ESP-NOW / Wi-Fi
 ┌──────┴──────┐   │
 │ DW3000 AoA  │   │
 │ 2 Antenna   │   │
 │ - Distance  │   │
 │ - AoA Angle │   │
 │ - UWB ID    │   │
 └─────────────┘   │
                   │
              Other Vehicles
```

---

## 3. 각 구성 요소의 역할

| 구성 요소 | 역할 |
|---|---|
| DW3000 AoA UWB 보드 | 차량 간 거리 및 방위각 측정 |
| ESP32-S3 | UWB 제어, 차량 상태 통신, ID 매칭, TDMA 관리 |
| ESP-NOW / Wi-Fi | 차량 ID, UWB ID, 속도, 플레툰 상태 등 의미 정보 공유 |
| Raspberry Pi | 상대좌표 계산, 센서퓨전, 자율주행 판단, 플레툰 제어 |
| UART / USB Serial | ESP32-S3에서 Raspberry Pi로 데이터 전달 |

---

## 4. 통신 역할 분리

이 프로젝트에서는 UWB와 Wi-Fi 계열 통신의 역할을 분리하는 것이 중요하다.

```text
ESP-NOW / Wi-Fi:
- 차량 ID
- UWB ID
- 차량 속도
- 목적지
- 플레툰 참여 여부
- 플레툰 ID
- 플레툰 내 순서
- 차량 역할: leader / follower

UWB AoA:
- 상대 차량 UWB ID
- 거리 r
- 방위각 θ
- 수신 품질
- timestamp
```

즉, Wi-Fi 계열 통신은 **의미 정보**를 전달하고, UWB는 **물리 위치 정보**를 측정한다.

```text
ESP-NOW = 이 차량이 누구인지 알려주는 통신
UWB AoA = 그 차량이 어디에 있는지 알려주는 센서
```

---

## 5. UWB ID와 차량 ID 매칭 구조

차량 상태 정보는 ESP-NOW로 공유되고, 거리 및 각도 정보는 UWB로 측정된다.  
두 정보를 연결하기 위한 핵심 키는 **UWB ID**이다.

예를 들어 ESP-NOW로 다음 정보를 받았다고 하자.

```text
vehicle_id = 102
uwb_id     = 0xA002
speed      = 1.2 m/s
platoon_id = 7
```

그리고 UWB AoA 측정 결과가 다음과 같이 들어온다.

```text
uwb_id     = 0xA002
distance   = 4.8 m
angle      = -6.5 deg
```

그러면 ESP32-S3는 UWB ID를 기준으로 두 데이터를 매칭한다.

```text
0xA002는 vehicle_id 102의 UWB ID
따라서 distance = 4.8 m, angle = -6.5 deg는 vehicle 102의 상대 위치 정보
```

---

## 6. 차량 테이블 구조

ESP32-S3 내부에서는 주변 차량 정보를 테이블 형태로 관리하는 것이 좋다.

```c
typedef struct {
    uint32_t vehicle_id;
    uint32_t uwb_id;

    float distance_m;
    float angle_deg;
    float rel_x_m;
    float rel_y_m;

    float speed_mps;
    float heading_deg;

    uint8_t platoon_enable;
    uint32_t platoon_id;
    uint8_t platoon_role;   // 0: none, 1: leader, 2: follower
    uint8_t platoon_index;

    uint32_t last_uwb_ms;
    uint32_t last_wifi_ms;

    float confidence;
    uint8_t valid;
} tracked_vehicle_t;
```

---

## 7. AoA 기반 상대 좌표 계산

DW3000 기반 2안테나 AoA 보드는 거리와 방위각을 제공한다.  
이를 차량 기준 좌표계로 변환하면 상대 차량의 2D 위치를 추정할 수 있다.

### 7.1 차량 기준 좌표계

```text
차량 진행 방향 = +X
차량 왼쪽     = +Y
차량 오른쪽   = -Y
차량 뒤쪽     = -X
```

```text
              +X, 전방
                ↑
                │
        +Y  ← 차량 A →  -Y
                │
                ↓
              -X, 후방
```

### 7.2 각도 정의

```text
θ = 0°    : 정면
θ > 0°   : 왼쪽
θ < 0°   : 오른쪽
θ = ±90° : 측면
```

### 7.3 상대 좌표 계산식

```text
rel_x = distance × cos(theta)
rel_y = distance × sin(theta)
```

예를 들어:

```text
distance = 5 m
angle    = +10°

rel_x = 5 × cos(10°) ≈ 4.92 m
rel_y = 5 × sin(10°) ≈ 0.87 m
```

따라서 상대 차량은 내 차량 기준으로 전방 4.92 m, 왼쪽 0.87 m 위치에 있다고 볼 수 있다.

---

## 8. ESP32-S3의 역할

ESP32-S3는 단순 통신 모듈이 아니라 UWB 센서 허브 역할을 수행한다.

주요 역할은 다음과 같다.

1. DW3000 AoA 보드 초기화
2. UWB ranging 및 AoA 측정 수행
3. 상대 UWB ID 수신
4. 거리 및 각도 데이터 수집
5. ESP-NOW로 차량 상태 패킷 송수신
6. UWB ID와 vehicle ID 매칭
7. 필요 시 상대 좌표 계산
8. Raspberry Pi로 최종 데이터 전달
9. TDMA 슬롯에 따라 UWB 송신 타이밍 제어

권장 역할 분리는 다음과 같다.

```text
ESP32-S3:
- UWB 원시 측정값 수집
- ESP-NOW 차량 정보 수집
- ID 매칭
- 패킷화

Raspberry Pi:
- 좌표 변환
- 필터링
- 센서퓨전
- 플레툰 제어 판단
- 로그 저장
```

---

## 9. Raspberry Pi의 역할

Raspberry Pi는 최종 판단 장치이다.

Raspberry Pi에서 수행할 작업은 다음과 같다.

1. ESP32-S3로부터 차량별 거리, 각도, 상태 정보 수신
2. 상대 좌표 계산 또는 보정
3. 시간 동기화 및 timestamp 확인
4. 오래된 데이터 제거
5. confidence 기반 유효성 판단
6. 플레툰 간격 제어
7. 횡방향 정렬 제어
8. 로그 저장
9. 시각화
10. 필요 시 카메라, IMU, 엔코더 정보와 센서퓨전

---

## 10. ESP-NOW 차량 상태 패킷

ESP-NOW로 주고받을 차량 상태 패킷 예시는 다음과 같다.

```c
typedef struct {
    uint32_t vehicle_id;
    uint32_t uwb_id;

    float speed_mps;
    float heading_deg;

    uint8_t platoon_enable;
    uint32_t platoon_id;
    uint8_t platoon_role;   // 0: none, 1: leader, 2: follower
    uint8_t platoon_index;

    uint32_t timestamp_ms;
    uint16_t seq;
} vehicle_status_packet_t;
```

권장 전송 주기는 다음과 같다.

```text
ESP-NOW 차량 상태 broadcast 주기: 50 ~ 100 ms
```

---

## 11. UWB AoA 측정 결과 구조

DW3000 AoA 보드에서 ESP32-S3로 전달되는 측정값은 다음과 같이 관리할 수 있다.

```c
typedef struct {
    uint32_t uwb_id;

    float distance_m;
    float angle_deg;

    int16_t rssi;
    float fp_power;
    float rx_power;

    uint32_t timestamp_ms;
    uint16_t seq;

    uint8_t valid;
} uwb_aoa_result_t;
```

여기서 중요한 값은 다음이다.

| 값 | 의미 |
|---|---|
| uwb_id | 측정 대상 UWB 고유 ID |
| distance_m | 상대 차량까지의 거리 |
| angle_deg | 상대 차량의 방위각 |
| rssi / fp_power / rx_power | 수신 품질 판단용 값 |
| timestamp_ms | 측정 시각 |
| valid | 측정 유효 여부 |

---

## 12. Raspberry Pi로 전달할 최종 패킷

초기 개발에서는 JSON 형식이 디버깅하기 편하다.

```json
{
  "self_vehicle_id": 101,
  "timestamp_ms": 123456,
  "targets": [
    {
      "vehicle_id": 102,
      "uwb_id": 40962,
      "distance_m": 4.8,
      "angle_deg": -6.5,
      "rel_x_m": 4.77,
      "rel_y_m": -0.54,
      "speed_mps": 1.2,
      "platoon_id": 7,
      "platoon_role": 2,
      "confidence": 0.91
    }
  ]
}
```

최종 구현에서는 바이너리 프레임이 더 적합하다.

```text
Header | Length | MsgType | Payload | CRC
```

---

## 13. TDMA 개념 정리

TDMA는 **Time Division Multiple Access**의 약자로, 여러 장치가 동시에 통신하지 않도록 시간을 나누어 사용하는 방식이다.

쉽게 말하면 다음과 같다.

```text
여러 차량이 동시에 UWB를 송신하면 충돌이 생기므로,
차량마다 UWB를 사용할 시간을 따로 배정하는 방식
```

---

## 14. 왜 TDMA가 필요한가?

차량이 여러 대 있을 때 각 차량이 동시에 UWB ranging을 시도하면 패킷 충돌이 발생할 수 있다.

예를 들어 차량 A, B, C가 동시에 UWB Poll을 보낸다면:

```text
A → B ranging 시도
B → C ranging 시도
C → A ranging 시도
```

이 경우 UWB 패킷이 겹칠 수 있고, 다음 문제가 발생한다.

- 거리 측정 실패
- 잘못된 거리값
- 응답 timeout
- 패킷 충돌
- AoA 측정 불안정

따라서 차량마다 통신 시간을 나눠주는 TDMA가 필요하다.

---

## 15. TDMA 기본 구조

예를 들어 TDMA 주기를 100 ms로 설정하고, 플레툰 차량이 4대라고 하면 다음처럼 슬롯을 나눌 수 있다.

```text
TDMA cycle = 100 ms
slot length = 25 ms
```

```text
시간 →
┌────────┬────────┬────────┬────────┐
│ A 슬롯 │ B 슬롯 │ C 슬롯 │ D 슬롯 │
└────────┴────────┴────────┴────────┘
0ms      25ms     50ms     75ms     100ms
```

각 차량은 자기 슬롯에서만 UWB ranging을 수행한다.

---

## 16. 플레툰에서의 TDMA 슬롯 예시

차량 구성이 다음과 같다고 하자.

```text
vehicle_id 101 = leader
vehicle_id 102 = follower 1
vehicle_id 103 = follower 2
vehicle_id 104 = follower 3
```

TDMA 슬롯은 다음과 같이 배정할 수 있다.

```text
slot 0: vehicle 101
slot 1: vehicle 102
slot 2: vehicle 103
slot 3: vehicle 104
```

시간표는 다음과 같다.

```text
100 ms cycle

0~25 ms      vehicle 101 UWB 측정
25~50 ms     vehicle 102 UWB 측정
50~75 ms     vehicle 103 UWB 측정
75~100 ms    vehicle 104 UWB 측정
```

---

## 17. Guard Time

TDMA에서는 차량 간 시간 기준이 완벽히 같지 않을 수 있다.  
따라서 슬롯 사이에 약간의 빈 시간을 두는 것이 좋다. 이 빈 시간을 **guard time**이라고 한다.

예:

```text
slot length = 25 ms
실제 UWB 사용 시간 = 20 ms
guard time = 5 ms
```

```text
0~20 ms      차량 A UWB 사용
20~25 ms     guard time
25~45 ms     차량 B UWB 사용
45~50 ms     guard time
```

Guard time은 슬롯이 겹치는 것을 방지한다.

---

## 18. TDMA 의사코드

ESP32-S3에서 사용할 수 있는 단순한 TDMA 의사코드는 다음과 같다.

```c
uint32_t cycle_ms = 100;
uint32_t slot_ms = 25;
uint8_t my_slot = 1;   // 예: vehicle 102

uint32_t now = millis();
uint32_t t = now % cycle_ms;

uint8_t current_slot = t / slot_ms;

if (current_slot == my_slot) {
    // 내 차례
    uwb_ranging_start();
} else {
    // 다른 차량 차례
    uwb_idle_or_receive();
}
```

---

## 19. Leader 기반 TDMA 관리

플레툰에서는 leader 차량이 TDMA 슬롯을 관리하는 것이 좋다.

Leader 차량의 역할:

- 플레툰 ID 생성
- 차량 참가 요청 수락
- 차량 순서 관리
- TDMA slot 번호 배정
- TDMA cycle 정보 전파
- 플레툰 목표 속도 및 간격 전파

Follower 차량의 역할:

- ESP-NOW로 leader 정보 수신
- 자신의 slot_id 확인
- 자기 슬롯에서만 UWB 송신
- 다른 슬롯에서는 수신 또는 idle

예시 슬롯 배정 패킷:

```json
{
  "platoon_id": 7,
  "vehicle_id": 102,
  "slot_id": 1,
  "cycle_ms": 100,
  "slot_ms": 25
}
```

---

## 20. 전체 동작 알고리즘

전체 시스템의 반복 동작은 다음과 같다.

```text
1. ESP32-S3 초기화
2. DW3000 AoA 보드 초기화
3. ESP-NOW 초기화
4. vehicle_id, uwb_id 설정
5. 플레툰 참여 여부 확인
6. leader가 TDMA slot 배정
7. 각 차량은 ESP-NOW로 상태 정보 broadcast
8. 각 차량은 자기 TDMA slot에서 UWB AoA ranging 수행
9. UWB 결과에서 uwb_id, distance, angle 수집
10. ESP-NOW 차량 테이블에서 uwb_id 매칭
11. vehicle_id별 distance, angle, speed, platoon 정보 통합
12. 상대 좌표 계산
13. confidence 계산
14. Raspberry Pi로 전송
15. Raspberry Pi에서 플레툰 제어 판단
```

---

## 21. ESP32-S3 내부 알고리즘 흐름

```text
while (1) {
    1. ESP-NOW 수신 패킷 처리
       - vehicle_id
       - uwb_id
       - speed
       - platoon_id
       - platoon_role
       - timestamp

    2. 현재 TDMA slot 확인
       - 현재 시간이 내 slot인지 확인

    3. 내 slot이면 UWB AoA ranging 수행
       - target UWB ID 선택
       - distance 측정
       - angle 측정

    4. UWB 결과와 차량 테이블 매칭
       - uwb_id 기준으로 vehicle_id 찾기

    5. 상대 좌표 계산
       - rel_x = distance × cos(angle)
       - rel_y = distance × sin(angle)

    6. 데이터 유효성 검사
       - timestamp 확인
       - 거리 범위 확인
       - 각도 범위 확인
       - 수신 품질 확인

    7. Raspberry Pi로 전송
}
```

---

## 22. Confidence 계산

UWB AoA는 반사, 차체 가림, NLOS 상황에서 오차가 커질 수 있다.  
따라서 거리와 각도값을 그대로 사용하지 말고 confidence를 함께 계산해야 한다.

간단한 confidence 계산 예시는 다음과 같다.

```text
confidence = 1.0

if distance_m <= 0 or distance_m > max_range:
    confidence -= 0.4

if abs(angle_deg) > 80:
    confidence -= 0.2

if uwb_age_ms > 200:
    confidence -= 0.3

if wifi_age_ms > 300:
    confidence -= 0.3

if signal_quality_bad:
    confidence -= 0.2
```

Confidence가 낮은 데이터는 제어에 직접 사용하지 않고, 보정 또는 무시해야 한다.

---

## 23. 상대 위치 필터링

AoA 기반 상대 좌표는 노이즈가 포함될 수 있으므로 필터링이 필요하다.

초기에는 저역통과 필터 형태로 시작할 수 있다.

```text
rel_x_filtered = alpha × rel_x_old + (1 - alpha) × rel_x_new
rel_y_filtered = alpha × rel_y_old + (1 - alpha) × rel_y_new
```

초기 권장값:

```text
alpha = 0.7 ~ 0.9
```

- 측정값이 너무 흔들리면 alpha를 높인다.
- 차량 움직임에 빠르게 반응해야 하면 alpha를 낮춘다.

---

## 24. 플레툰 제어에 필요한 값

Raspberry Pi가 플레툰 제어에 사용해야 하는 핵심 값은 다음과 같다.

```text
앞차 vehicle_id
앞차 distance
앞차 angle
앞차 rel_x
앞차 rel_y
앞차 speed
내 차량 speed
목표 간격 target_distance
confidence
```

간격 제어 오차는 다음과 같이 계산할 수 있다.

```text
e_d = target_distance - rel_x
```

횡방향 정렬 오차는 다음과 같이 계산할 수 있다.

```text
e_y = rel_y
```

예:

```text
target_distance = 2.0 m
rel_x = 2.6 m
rel_y = -0.2 m

e_d = 2.0 - 2.6 = -0.6 m
→ 앞차와 너무 멀다. 가속 필요.

e_y = -0.2 m
→ 앞차가 오른쪽에 있다. 오른쪽 정렬 필요.
```

---

## 25. 예상 문제와 해결 방향

### 25.1 UWB AoA 각도 오차

문제:

- 반사파
- 차체 가림
- 안테나 배치 불량
- NLOS 상황

해결:

- 수신 품질 기반 confidence 계산
- moving average 또는 low-pass filter 적용
- 이전 위치와 비교하여 튀는 값 제거
- Raspberry Pi에서 IMU, 엔코더, 카메라 정보와 센서퓨전

---

### 25.2 전방/후방 모호성

문제:

2안테나 AoA 구조에서는 조건에 따라 전방과 후방 구분이 애매할 수 있다.

해결:

- 플레툰 순서 정보 사용
- 차량 heading 사용
- 이전 위치 추정값 사용
- 앞차로 예상되는 차량은 전방 각도 범위만 허용

예:

```text
앞차 expected angle range = -45° ~ +45°
이 범위를 벗어나면 confidence 감소
```

---

### 25.3 여러 차량 UWB 충돌

문제:

여러 차량이 동시에 UWB ranging을 수행하면 패킷 충돌이 발생한다.

해결:

- TDMA slot 사용
- leader가 slot 배정
- guard time 추가
- 차량 수 증가 시 cycle length 조정

---

### 25.4 Wi-Fi 정보와 UWB 정보의 시간 불일치

문제:

ESP-NOW로 받은 차량 상태는 100 ms 전 정보이고, UWB 거리 측정은 현재 정보일 수 있다.

해결:

- 모든 패킷에 timestamp 포함
- sequence number 포함
- 오래된 정보 제거
- Raspberry Pi에서 시간 보정

권장 기준:

```text
UWB 정보 age > 200 ms → invalid 또는 confidence 감소
Wi-Fi 정보 age > 300 ms → invalid 또는 confidence 감소
```

---

## 26. 추천 개발 순서

### 1단계: ESP32-S3와 Raspberry Pi 유선 통신

먼저 ESP32-S3가 Raspberry Pi로 데이터를 보내는 구조를 만든다.

```text
ESP32-S3 → USB Serial / UART → Raspberry Pi
```

초기 데이터:

```text
vehicle_id
timestamp
dummy distance
dummy angle
```

---

### 2단계: ESP-NOW 차량 상태 송수신

ESP32-S3 두 대로 차량 상태 패킷을 주고받는다.

```text
vehicle_id
uwb_id
speed
platoon_id
platoon_role
```

---

### 3단계: DW3000 AoA 보드 거리 및 각도 측정

ESP32-S3와 DW3000 AoA 보드를 연결하고 거리와 각도값을 읽는다.

```text
ESP32-S3 ↔ SPI ↔ DW3000 AoA Board
```

확인할 값:

```text
distance
angle
uwb_id
signal quality
```

---

### 4단계: UWB ID와 vehicle ID 매칭

ESP-NOW로 받은 UWB ID와 UWB 측정 결과의 UWB ID를 비교한다.

```text
ESP-NOW: uwb_id = 0xA002, vehicle_id = 102
UWB:     uwb_id = 0xA002, distance = 4.8 m, angle = -6.5°

→ vehicle 102 위치 정보로 확정
```

---

### 5단계: TDMA 적용

차량이 3대 이상이면 UWB 충돌을 막기 위해 TDMA를 적용한다.

```text
100 ms cycle
slot 0: leader
slot 1: follower 1
slot 2: follower 2
slot 3: follower 3
```

---

### 6단계: Raspberry Pi에서 상대 위치 시각화

처음에는 제어보다 시각화를 먼저 구현한다.

```text
vehicle 102: x = 3.2 m, y = -0.4 m
vehicle 103: x = 6.1 m, y = +0.2 m
```

---

### 7단계: 플레툰 제어 연결

상대 좌표를 이용해 간격 제어와 횡방향 정렬 제어를 수행한다.

```text
rel_x → 앞차와의 거리 제어
rel_y → 대열 정렬 제어
```

---

## 27. 최종 추천 구조

현재 프로젝트에서는 다음 구조를 추천한다.

```text
통신 방식:
ESP-NOW broadcast

UWB 방식:
DW3000 기반 AoA ranging

위치 추정:
distance + angle 기반 2D 상대좌표 계산

다중 차량 충돌 방지:
TDMA slot 구조

ID 매칭:
vehicle_id ↔ uwb_id 테이블

Raspberry Pi 연결:
USB Serial 또는 UART

Raspberry Pi 역할:
상대 위치 필터링, 플레툰 제어, 로그 저장
```

---

## 28. 최종 결론

본 프로젝트의 핵심 구조는 다음과 같이 정리할 수 있다.

```text
ESP-NOW로 “차량이 누구인지” 확인하고,
UWB AoA로 “그 차량이 어디에 있는지” 측정한 뒤,
UWB ID를 기준으로 두 정보를 매칭하여,
Raspberry Pi에 상대 위치 정보를 전달한다.
```

따라서 프로젝트 제목은 다음과 같이 표현할 수 있다.

```text
ESP32-S3와 DW3000 AoA UWB를 이용한
ID 매칭 기반 V2V 상대 위치 추정 시스템
```

또는:

```text
ESP-NOW 차량 상태 공유와 UWB AoA 거리·방위각 측정을 결합한
플레툰 자율주행용 상대 위치 추정 구조
```

이 구조는 초기 프로토타입부터 실제 플레툰 제어까지 확장 가능하며, 차량 수가 증가할 경우 TDMA를 통해 UWB 충돌을 제어할 수 있다.
