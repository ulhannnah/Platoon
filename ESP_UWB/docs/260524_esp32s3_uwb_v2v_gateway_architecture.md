# ESP32-S3 기준 UWB V2V 게이트웨이 구조 정리

## 1. 프로젝트 내 ESP32-S3의 역할

본 프로젝트는 플레툰 자율주행 차량 간 통신 및 상대 위치 추정을 목표로 한다. 각 차량에는 **ESP32-S3**, **DW3000 기반 2안테나 AoA UWB 보드**, **Raspberry Pi**가 탑재된다.

이 중 ESP32-S3는 자율주행 판단을 직접 수행하는 메인 컴퓨터가 아니라, 다음 역할을 담당하는 **V2V 센서/통신 게이트웨이**이다.

```text
ESP32-S3의 주요 역할

1. DW3000 UWB AoA 보드 제어
2. UWB를 통한 거리, 각도, UWB_ID 수집
3. ESP-NOW를 통한 차량 상태 정보 송수신
4. UWB_ID와 vehicle_id 매칭
5. TDMA 기반 UWB ranging 시간 관리
6. 주변 차량 테이블 유지
7. Raspberry Pi로 최종 상대 차량 정보 전달
```

즉 ESP32-S3는 **UWB에서 얻은 물리 위치 정보**와 **ESP-NOW에서 받은 차량 의미 정보**를 합쳐서 Raspberry Pi로 전달하는 역할을 한다.

---

## 2. 전체 시스템에서 ESP32-S3의 위치

```text
                ┌──────────────────────────────┐
                │        Raspberry Pi + ROS      │
                │ - 상대좌표 추정                │
                │ - 플레툰 제어                  │
                │ - 자율주행 판단                │
                │ - 로그 저장                    │
                └──────────────▲───────────────┘
                               │ USB Serial / UART
                               │
┌──────────────────────────────┴──────────────────────────────┐
│                         ESP32-S3                              │
│                                                              │
│  ┌──────────────┐   ┌──────────────┐   ┌──────────────────┐ │
│  │ UWB Manager  │   │ ESP-NOW Comm │   │ Pi Serial Bridge │ │
│  │ DW3000 제어   │   │ 차량정보 송수신│   │ Pi로 데이터 전송  │ │
│  └──────▲───────┘   └──────▲───────┘   └────────▲─────────┘ │
│         │                  │                    │           │
│  ┌──────┴──────────────────┴────────────────────┴────────┐ │
│  │             Vehicle Table / ID Matching                 │ │
│  │ vehicle_id, uwb_id, distance, angle, speed, platoon 정보 │ │
│  └────────────────────────▲───────────────────────────────┘ │
│                           │                                 │
│  ┌────────────────────────┴───────────────────────────────┐ │
│  │                  TDMA Scheduler                         │ │
│  │           내 slot에서만 UWB ranging 수행                 │ │
│  └────────────────────────────────────────────────────────┘ │
└──────────────▲──────────────────────────────▲───────────────┘
               │ SPI                          │ ESP-NOW
               │                              │
        ┌──────┴───────┐              ┌───────┴───────┐
        │ DW3000 AoA   │              │ Other ESP32   │
        │ UWB Board    │              │ Vehicles      │
        └──────────────┘              └───────────────┘
```

---

## 3. ESP32-S3 내부 소프트웨어 모듈 구조

ESP32 펌웨어는 다음과 같이 모듈화하는 것이 좋다.

```text
main
├── uwb_manager
├── espnow_manager
├── tdma_scheduler
├── vehicle_table
├── id_matcher
├── pi_serial_bridge
└── system_monitor
```

각 모듈은 독립적인 역할을 가지며, 최종적으로 `vehicle_table`을 중심으로 데이터가 합쳐진다.

---

# 4. 모듈별 상세 역할

## 4.1 `uwb_manager`

`uwb_manager`는 DW3000 기반 UWB AoA 보드를 제어하는 모듈이다.

### 역할

```text
1. DW3000 초기화
2. SPI 통신 설정
3. UWB 채널, preamble, data rate 설정
4. UWB ranging 패킷 송수신
5. Poll / Response / Final 패킷 처리
6. 거리 계산에 필요한 timestamp 읽기
7. AoA 또는 PDoA 관련 측정값 읽기
8. 상대 UWB_ID 추출
9. UWB 측정 결과 생성
```

### ESP32 → DW3000

ESP32가 DW3000에 보내는 데이터 또는 명령은 다음과 같다.

```text
- 초기화 명령
- 레지스터 설정값
- UWB 채널 설정
- preamble 설정
- 송신할 UWB 패킷
- 송신 시작 명령
- 수신 대기 명령
- 인터럽트 클리어 명령
```

### DW3000 → ESP32

DW3000이 ESP32에 제공하는 정보는 다음과 같다.

```text
- 수신 완료 인터럽트
- 수신한 UWB 패킷
- TX timestamp
- RX timestamp
- PDoA / AoA 관련 측정값
- RSSI / 수신 전력 / first path power
- 수신 품질 정보
```

### UWB 측정 결과 구조체 예시

```c
typedef struct {
    uint32_t target_uwb_id;

    float distance_m;
    float angle_deg;

    float fp_power;
    float rx_power;
    int16_t rssi;

    uint32_t timestamp_ms;
    uint16_t seq;

    uint8_t valid;
} uwb_result_t;
```

---

## 4.2 `espnow_manager`

`espnow_manager`는 차량 간 상태 정보를 ESP-NOW로 주고받는 모듈이다.

UWB가 물리 위치 정보를 담당한다면, ESP-NOW는 차량의 의미 정보를 담당한다.

### 역할

```text
1. ESP-NOW 초기화
2. 주변 차량으로 내 차량 상태 broadcast
3. 주변 차량 상태 수신
4. vehicle_id, uwb_id, platoon_id 수신
5. 차량 속도, heading, 플레툰 상태 수신
6. 수신된 정보를 vehicle_table에 반영
```

### ESP-NOW로 송수신하는 정보

```text
- vehicle_id
- uwb_id
- platoon_id
- platoon_enable
- platoon_role
- platoon_index
- speed_mps
- heading_deg
- timestamp_ms
- sequence number
```

### 차량 상태 패킷 예시

```c
typedef struct {
    uint32_t vehicle_id;
    uint32_t uwb_id;

    uint32_t platoon_id;
    uint8_t platoon_enable;
    uint8_t platoon_role;   // 0: none, 1: leader, 2: follower
    uint8_t platoon_index;

    float speed_mps;
    float heading_deg;

    uint32_t timestamp_ms;
    uint16_t seq;
} vehicle_status_packet_t;
```

ESP-NOW 패킷은 다음 의미를 가진다.

```text
나는 vehicle_id 102이고,
내 UWB_ID는 0xA002이고,
플레툰 7번에 참여 중이고,
현재 속도는 1.2 m/s이다.
```

---

## 4.3 `vehicle_table`

`vehicle_table`은 ESP32 내부에서 주변 차량 정보를 저장하는 핵심 데이터 구조이다.

이 테이블은 두 종류의 정보를 하나로 합친다.

```text
ESP-NOW로 받은 정보:
vehicle_id, uwb_id, speed, heading, platoon 상태

UWB로 측정한 정보:
uwb_id, distance, angle, signal quality
```

### 차량 테이블 구조체 예시

```c
#define MAX_VEHICLES 16

typedef struct {
    uint32_t vehicle_id;
    uint32_t uwb_id;

    float distance_m;
    float angle_deg;
    float rel_x_m;
    float rel_y_m;

    float speed_mps;
    float heading_deg;

    uint32_t platoon_id;
    uint8_t platoon_enable;
    uint8_t platoon_role;
    uint8_t platoon_index;

    uint32_t last_uwb_ms;
    uint32_t last_espnow_ms;

    float confidence;
    uint8_t valid;
} tracked_vehicle_t;
```

---

## 4.4 `id_matcher`

`id_matcher`는 UWB 측정 결과와 ESP-NOW 차량 정보를 연결하는 모듈이다.

핵심은 `uwb_id`이다.

```text
UWB 결과:
uwb_id = 0xA002
거리 = 3.42 m
각도 = -7.5°

ESP-NOW 차량 정보:
vehicle_id = 102
uwb_id = 0xA002
속도 = 1.2 m/s

매칭 결과:
vehicle_id 102는 내 기준 거리 3.42 m, 각도 -7.5°에 위치한다.
```

### 매칭 동작 예시

```c
for each tracked_vehicle:
    if tracked_vehicle.uwb_id == uwb_result.target_uwb_id:
        tracked_vehicle.distance_m = uwb_result.distance_m;
        tracked_vehicle.angle_deg = uwb_result.angle_deg;
        tracked_vehicle.rel_x_m = distance * cos(angle);
        tracked_vehicle.rel_y_m = distance * sin(angle);
        tracked_vehicle.last_uwb_ms = now;
```

---

## 4.5 `tdma_scheduler`

`tdma_scheduler`는 여러 차량이 동시에 UWB ranging을 시도하지 않도록 시간 슬롯을 관리하는 모듈이다.

UWB ranging은 Poll, Response, Final 등의 패킷을 주고받기 때문에 여러 차량이 동시에 송신하면 충돌이 발생할 수 있다. 따라서 각 차량에 UWB 사용 시간을 배정해야 한다.

### 예시

차량 4대가 있을 때 100 ms를 하나의 cycle로 잡으면 다음과 같이 나눌 수 있다.

```text
TDMA cycle = 100 ms

0~25 ms      vehicle 101
25~50 ms     vehicle 102
50~75 ms     vehicle 103
75~100 ms    vehicle 104
```

### Guard time

실제 구현에서는 slot 전체를 UWB에 사용하지 않고, 일부는 시간 오차를 흡수하는 guard time으로 둔다.

```text
slot length = 25 ms
실제 UWB 사용 시간 = 20 ms
guard time = 5 ms
```

### TDMA 판단 코드 예시

```c
uint32_t cycle_ms = 100;
uint32_t slot_ms = 25;
uint8_t my_slot = platoon_index;

uint32_t t = millis() % cycle_ms;
uint8_t current_slot = t / slot_ms;

if (current_slot == my_slot) {
    uwb_ranging_start();
} else {
    uwb_rx_idle();
}
```

---

## 4.6 `pi_serial_bridge`

`pi_serial_bridge`는 ESP32가 정리한 차량 정보를 Raspberry Pi로 전달하는 모듈이다.

초기 개발 단계에서는 JSON 형식이 디버깅에 유리하다.

### JSON 출력 예시

```json
{
  "self_vehicle_id": 101,
  "timestamp_ms": 123456,
  "targets": [
    {
      "vehicle_id": 102,
      "uwb_id": 40962,
      "distance_m": 3.42,
      "angle_deg": -7.5,
      "rel_x_m": 3.39,
      "rel_y_m": -0.45,
      "speed_mps": 1.2,
      "platoon_id": 7,
      "confidence": 0.91
    }
  ]
}
```

최종 구현에서는 데이터 크기와 속도를 고려해 바이너리 프레임으로 바꿀 수 있다.

```text
Header | Length | MsgType | Payload | CRC
```

---

# 5. ESP32 기준 데이터 흐름

## 5.1 ESP-NOW 데이터 흐름

```text
다른 차량 ESP32
        ↓ ESP-NOW
내 ESP32-S3
        ↓
espnow_manager
        ↓
vehicle_table 갱신
```

수신 정보:

```text
vehicle_id
uwb_id
speed
heading
platoon_id
platoon_role
platoon_index
timestamp
```

---

## 5.2 UWB 데이터 흐름

```text
DW3000 AoA 보드
        ↓ SPI / IRQ
내 ESP32-S3
        ↓
uwb_manager
        ↓
uwb_result 생성
        ↓
id_matcher
        ↓
vehicle_table 갱신
```

측정 정보:

```text
target_uwb_id
distance_m
angle_deg
rx_power
fp_power
timestamp
```

---

## 5.3 Raspberry Pi 전송 흐름

```text
vehicle_table
        ↓
pi_serial_bridge
        ↓ USB Serial / UART
Raspberry Pi
        ↓
ROS2 serial node
        ↓
/uwb/targets
/platoon/vehicles
/platoon/control_cmd
```

---

# 6. ESP32 FreeRTOS Task 구조

ESP-IDF 기반으로 구현한다면 FreeRTOS task를 다음과 같이 나누는 것이 좋다.

```text
ESP32 FreeRTOS Tasks

1. task_uwb
   - DW3000 제어
   - UWB ranging 처리
   - 거리/각도 결과 생성

2. task_espnow
   - ESP-NOW 송수신 처리
   - 주변 차량 상태 수신
   - 내 차량 상태 broadcast

3. task_tdma
   - TDMA cycle 관리
   - 현재 slot 확인
   - UWB 송신 가능 여부 판단

4. task_pi_tx
   - Raspberry Pi로 주기적 전송
   - JSON 또는 binary frame 생성

5. task_monitor
   - timeout 확인
   - 오래된 차량 정보 삭제
   - 오류 상태 관리
```

Task 간 데이터 공유는 `vehicle_table`을 중심으로 이루어진다.

```text
┌──────────────────────┐
│ task_espnow           │
│ 차량 상태 송수신        │
└──────────┬───────────┘
           │
           ▼
┌──────────────────────┐
│ vehicle_table         │
└──────────▲───────────┘
           │
┌──────────┴───────────┐
│ task_uwb              │
│ UWB 거리/각도 측정      │
└──────────┬───────────┘
           │
           ▼
┌──────────────────────┐
│ id_matcher            │
└──────────┬───────────┘
           │
           ▼
┌──────────────────────┐
│ task_pi_tx            │
│ Raspberry Pi 전송      │
└──────────────────────┘
```

---

# 7. ESP32 상태 머신

ESP32 전체 동작은 다음 상태 머신으로 구성할 수 있다.

```text
BOOT
 ↓
INIT_UWB
 ↓
INIT_ESPNOW
 ↓
WAIT_PLATOON_INFO
 ↓
RUN_NORMAL
 ↓
ERROR_RECOVERY
```

## 상태별 역할

| 상태 | 역할 |
|---|---|
| BOOT | 전원 인가 후 기본 초기화 |
| INIT_UWB | DW3000 초기화 및 SPI 설정 |
| INIT_ESPNOW | ESP-NOW 초기화 및 peer/broadcast 설정 |
| WAIT_PLATOON_INFO | leader, platoon_id, slot 정보 대기 |
| RUN_NORMAL | 정상 동작, UWB ranging 및 ESP-NOW 송수신 |
| ERROR_RECOVERY | UWB timeout, ESP-NOW 오류, 테이블 불일치 복구 |

## RUN_NORMAL 내부 동작

```text
1. ESP-NOW 수신 정보 업데이트
2. TDMA slot 확인
3. 내 slot이면 UWB ranging 수행
4. UWB 결과와 vehicle_table 매칭
5. 오래된 차량 정보 제거
6. Raspberry Pi로 결과 전송
```

---

# 8. ESP32와 Raspberry Pi의 역할 분리

## ESP32-S3에서 처리할 것

```text
- DW3000 제어
- UWB 거리/각도 측정
- ESP-NOW 차량 상태 송수신
- UWB_ID와 vehicle_id 매칭
- 간단한 rel_x, rel_y 계산
- timestamp 관리
- confidence 1차 계산
- Raspberry Pi로 패킷 전송
```

## Raspberry Pi에서 처리할 것

```text
- ROS2 토픽 변환
- 상대 좌표 필터링
- EKF 또는 센서퓨전
- 플레툰 제어
- 모터 제어 명령 생성
- 로그 저장
- 시각화
```

따라서 ESP32는 **실시간 통신 허브**, Raspberry Pi는 **판단 및 제어 컴퓨터**로 분리하는 것이 좋다.

---

# 9. ESP-IDF 프로젝트 파일 구조 추천

ESP32 펌웨어 프로젝트는 다음과 같이 구성할 수 있다.

```text
esp32_platoon_uwb/
├── CMakeLists.txt
├── sdkconfig
├── main/
│   ├── CMakeLists.txt
│   ├── main.c
│   │
│   ├── app_config.h
│   ├── packet_defs.h
│   │
│   ├── vehicle_table.h
│   ├── vehicle_table.c
│   │
│   ├── uwb_manager.h
│   ├── uwb_manager.c
│   │
│   ├── dw3000_port.h
│   ├── dw3000_port.c
│   │
│   ├── espnow_manager.h
│   ├── espnow_manager.c
│   │
│   ├── tdma_scheduler.h
│   ├── tdma_scheduler.c
│   │
│   ├── id_matcher.h
│   ├── id_matcher.c
│   │
│   ├── pi_serial_bridge.h
│   ├── pi_serial_bridge.c
│   │
│   ├── system_monitor.h
│   └── system_monitor.c
```

---

# 10. 최종 요약

ESP32-S3는 본 프로젝트에서 다음 구조를 가진다.

```text
1. 아래쪽:
   DW3000 AoA 보드와 SPI 연결

2. 옆쪽:
   다른 차량 ESP32와 ESP-NOW 연결

3. 위쪽:
   Raspberry Pi와 USB Serial 또는 UART 연결

4. 내부:
   UWB 결과와 ESP-NOW 차량 정보를 vehicle_table에서 매칭

5. 출력:
   vehicle_id별 distance, angle, rel_x, rel_y, speed, platoon 상태를 Raspberry Pi로 전달
```

한 줄로 정리하면 다음과 같다.

> ESP32-S3는 UWB AoA 센서와 ESP-NOW 차량 네트워크를 연결하고, UWB_ID 기반으로 물리 위치 정보와 차량 상태 정보를 매칭해서 Raspberry Pi로 전달하는 V2V 센서 게이트웨이 구조를 가진다.

