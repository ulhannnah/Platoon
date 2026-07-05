# ESP32-S3 + DW3000 AoA 기반 플레툰 V2V 패킷 정의

작성일: 2026-07-05  
프로젝트: ESP32-S3 + DW3000 AoA UWB 기반 플레툰 자율주행 V2V 통신 구조

---

## 1. 프로젝트 통신 구조 요약

본 프로젝트는 차량마다 **ESP32-S3**와 **DW3000 기반 2안테나 AoA UWB 보드**를 탑재하고, 차량 간 상태 정보 공유와 상대 위치 추정을 수행하는 구조이다.

전체 역할은 다음과 같이 나눈다.

```text
ESP32-S3
- ESP-NOW 기반 차량 간 상태 정보 송수신
- DW3000 AoA UWB 보드 제어
- UWB_ID와 vehicle_id 매칭
- TDMA 기반 UWB ranging 시간 관리
- Raspberry Pi로 최종 상대 차량 정보 전달

DW3000 AoA UWB
- 차량 간 거리 측정
- 방위각 AoA 측정
- UWB_ID 기반 상대 차량 식별

Raspberry Pi + ROS
- ESP32에서 받은 상대 차량 정보 수신
- ROS topic 변환
- 상대 위치 필터링
- 플레툰 제어 및 자율주행 판단
```

차량 간 정보 교환은 크게 두 종류로 나뉜다.

```text
ESP-NOW:
- 차량 ID
- UWB ID
- 속도
- heading
- 플레툰 ID
- 플레툰 역할
- 플레툰 순서
- 제어 명령
- 긴급 상태

UWB AoA:
- target UWB ID
- 거리 distance
- 방위각 angle
- 수신 품질
```

핵심은 다음과 같다.

```text
ESP-NOW로 “이 차량이 누구인지” 공유하고,
UWB AoA로 “그 차량이 어디에 있는지” 측정한다.
두 정보는 UWB_ID를 기준으로 매칭한다.
```

---

## 2. 패킷 분류

차량 간 ESP-NOW로 주고받을 패킷은 다음과 같이 나눈다.

| 패킷 이름 | 역할 | 초기 구현 여부 |
|---|---|---|
| Vehicle Status Packet | 차량 기본 상태 broadcast | 필수 |
| Platoon Info Packet | 플레툰 구성 정보 broadcast | 필수 |
| TDMA Sync Packet | UWB ranging 시간 슬롯 동기화 | 2단계 |
| Control Command Packet | leader가 follower에게 제어 목표 전달 | 3단계 |
| Emergency Packet | 급정지, 통신 이상, 충돌 위험 알림 | 4단계 |
| ACK Packet | 신뢰성 확인용 응답 | 선택 |

초기 구현에서는 아래 두 개를 먼저 구현한다.

```text
1. Vehicle Status Packet
2. Platoon Info Packet
```

이 두 패킷만 있어도 다음 동작이 가능하다.

```text
1. 각 차량이 자신의 vehicle_id와 uwb_id를 broadcast
2. 주변 차량의 vehicle_id ↔ uwb_id 테이블 생성
3. leader가 platoon_id와 대열 순서를 broadcast
4. 각 차량이 자기 platoon_index 확인
5. UWB AoA 결과의 target_uwb_id를 vehicle_id와 매칭
6. Raspberry Pi로 상대 차량 정보 전달
```

---

## 3. 공통 패킷 헤더

모든 ESP-NOW 패킷은 공통 헤더를 갖는 구조를 추천한다.

```c
typedef struct {
    uint8_t  magic;            // 패킷 시작 확인용, 예: 0xA5
    uint8_t  version;          // 프로토콜 버전
    uint8_t  msg_type;         // 패킷 종류
    uint8_t  header_len;       // 헤더 길이

    uint32_t src_vehicle_id;   // 송신 차량 ID
    uint32_t dst_vehicle_id;   // 수신 차량 ID, broadcast = 0xFFFFFFFF

    uint32_t timestamp_ms;     // 송신 시각
    uint16_t seq;              // sequence number
    uint16_t payload_len;      // payload 크기
} packet_header_t;
```

### 메시지 타입 정의

```c
#define MSG_VEHICLE_STATUS   0x01
#define MSG_PLATOON_INFO     0x02
#define MSG_TDMA_SYNC        0x03
#define MSG_CONTROL_COMMAND  0x04
#define MSG_EMERGENCY        0x05
#define MSG_ACK              0x06
```

### 공통 헤더 필드 설명

| 필드 | 설명 |
|---|---|
| magic | 패킷 유효성 확인용 고정값 |
| version | 패킷 프로토콜 버전 |
| msg_type | payload 종류 구분 |
| header_len | 헤더 길이 |
| src_vehicle_id | 송신 차량 번호 |
| dst_vehicle_id | 수신 차량 번호, broadcast 시 0xFFFFFFFF |
| timestamp_ms | 송신 시각 |
| seq | 패킷 순서 번호 |
| payload_len | payload 크기 |

---

## 4. Vehicle Status Packet

### 목적

Vehicle Status Packet은 각 차량이 자신의 기본 상태를 주변 차량에게 주기적으로 알리는 패킷이다.

이 패킷에서 가장 중요한 필드는 다음 두 개이다.

```text
vehicle_id
uwb_id
```

UWB AoA 측정 결과는 `target_uwb_id`로 나오기 때문에, ESP-NOW를 통해 미리 `vehicle_id ↔ uwb_id` 관계를 공유해야 한다.

### 송신 주기

```text
50 ms ~ 100 ms
```

### Payload 구조체

```c
typedef struct {
    uint32_t vehicle_id;
    uint32_t uwb_id;

    uint32_t platoon_id;
    uint8_t  platoon_enable;    // 0: not joined, 1: joined
    uint8_t  platoon_role;      // 0: none, 1: leader, 2: follower
    uint8_t  platoon_index;     // 대열 내 순서
    uint8_t  driving_state;     // 0: stop, 1: manual, 2: auto, 3: platoon

    float speed_mps;            // 현재 속도
    float heading_deg;          // 차량 진행 방향

    uint32_t timestamp_ms;
    uint16_t seq;
} vehicle_status_packet_t;
```

### 필드 설명

| 필드 | 설명 |
|---|---|
| vehicle_id | 차량 논리 ID |
| uwb_id | DW3000 UWB ID |
| platoon_id | 참여 중인 플레툰 ID |
| platoon_enable | 플레툰 참여 여부 |
| platoon_role | leader/follower 역할 |
| platoon_index | 플레툰 내 순서 |
| driving_state | 주행 상태 |
| speed_mps | 현재 속도 |
| heading_deg | 차량 진행 방향 |
| timestamp_ms | 데이터 생성 시각 |
| seq | 패킷 순서 번호 |

### 예시

```text
vehicle_id = 102
uwb_id = 0xA002
platoon_id = 7
platoon_enable = 1
platoon_role = 2
platoon_index = 1
speed_mps = 1.2
heading_deg = 3.5
```

해석:

```text
102번 차량은 UWB ID 0xA002를 가지고 있고,
플레툰 7번에 follower로 참여 중이며,
대열 내 순서는 1번이고,
현재 속도는 1.2 m/s이다.
```

---

## 5. Platoon Info Packet

### 목적

Platoon Info Packet은 플레툰 구성 정보를 공유하기 위한 패킷이다.  
주로 leader 차량이 broadcast한다.

포함 정보는 다음과 같다.

```text
플레툰 ID
leader 차량 ID
대열 차량 수
목표 속도
목표 간격
플레툰 멤버 목록
각 멤버의 UWB ID
```

### 송신 주기

```text
100 ms ~ 500 ms
```

플레툰 구성 변경 시에는 즉시 송신한다.

### Payload 구조체

```c
#define MAX_PLATOON_MEMBERS 8

typedef struct {
    uint32_t platoon_id;
    uint32_t leader_vehicle_id;

    uint8_t member_count;
    uint8_t platoon_state;      // 0: forming, 1: active, 2: leaving, 3: emergency
    uint8_t reserved0;
    uint8_t reserved1;

    float target_speed_mps;
    float target_gap_m;

    uint32_t member_vehicle_id[MAX_PLATOON_MEMBERS];
    uint32_t member_uwb_id[MAX_PLATOON_MEMBERS];

    uint32_t timestamp_ms;
    uint16_t seq;
} platoon_info_packet_t;
```

### 필드 설명

| 필드 | 설명 |
|---|---|
| platoon_id | 플레툰 그룹 ID |
| leader_vehicle_id | leader 차량 ID |
| member_count | 플레툰 참여 차량 수 |
| platoon_state | 플레툰 상태 |
| target_speed_mps | 목표 속도 |
| target_gap_m | 차량 간 목표 간격 |
| member_vehicle_id | 플레툰 멤버 차량 ID 목록 |
| member_uwb_id | 각 멤버의 UWB ID 목록 |
| timestamp_ms | 데이터 생성 시각 |
| seq | 패킷 순서 번호 |

### 예시

```text
platoon_id = 7
leader_vehicle_id = 101
member_count = 3
target_speed_mps = 1.0
target_gap_m = 2.0

member_vehicle_id = [101, 102, 103]
member_uwb_id     = [0xA001, 0xA002, 0xA003]
```

해석:

```text
플레툰 7번은 101번 차량이 leader이고,
101, 102, 103 차량이 참여 중이며,
목표 속도는 1.0 m/s,
목표 차량 간격은 2.0 m이다.
```

---

## 6. TDMA Sync Packet

### 목적

TDMA Sync Packet은 UWB ranging 충돌을 막기 위해, 각 차량이 언제 UWB를 사용할지 정하는 패킷이다.

차량이 3대 이상일 때 필요성이 커진다.

```text
여러 차량이 동시에 UWB ranging을 시도하면 패킷 충돌이 발생할 수 있다.
TDMA는 차량마다 UWB 송신 시간을 나눠주는 방식이다.
```

### 송신 주기

```text
100 ms ~ 500 ms
또는 플레툰 구성 변경 시 즉시 송신
```

### Payload 구조체

```c
#define MAX_TDMA_SLOTS 8

typedef struct {
    uint32_t platoon_id;
    uint32_t sync_time_ms;

    uint16_t cycle_ms;          // 예: 100 ms
    uint16_t slot_ms;           // 예: 20 ms
    uint16_t guard_ms;          // 예: 5 ms
    uint8_t  slot_count;
    uint8_t  reserved0;

    uint32_t slot_vehicle_id[MAX_TDMA_SLOTS];
    uint32_t slot_uwb_id[MAX_TDMA_SLOTS];
} tdma_sync_packet_t;
```

### 필드 설명

| 필드 | 설명 |
|---|---|
| platoon_id | 대상 플레툰 ID |
| sync_time_ms | 기준 동기화 시각 |
| cycle_ms | TDMA 전체 주기 |
| slot_ms | 각 차량 슬롯 길이 |
| guard_ms | 슬롯 사이 보호 시간 |
| slot_count | 사용 슬롯 수 |
| slot_vehicle_id | 슬롯별 차량 ID |
| slot_uwb_id | 슬롯별 UWB ID |

### 예시

```text
cycle_ms = 100
slot_ms = 20
guard_ms = 5
slot_count = 4

slot 0 = vehicle 101
slot 1 = vehicle 102
slot 2 = vehicle 103
slot 3 = vehicle 104
```

해석:

```text
100 ms 주기 안에서
101 → 102 → 103 → 104 순서로 UWB ranging을 수행한다.
각 차량은 자기 slot에서만 UWB 송신을 수행한다.
```

---

## 7. Control Command Packet

### 목적

Control Command Packet은 leader가 follower에게 목표 속도, 목표 간격, 대열 유지 명령 등을 전달하는 패킷이다.

ESP32가 직접 모터 제어를 하지 않는 구조에서는 이 패킷을 Raspberry Pi로 전달하고, Raspberry Pi가 ROS 기반 제어 노드에서 처리한다.

### 송신 주기

```text
20 ms ~ 100 ms
```

### Payload 구조체

```c
typedef struct {
    uint32_t platoon_id;
    uint32_t target_vehicle_id;

    uint8_t  command_type;      // 0: none, 1: follow, 2: stop, 3: leave, 4: emergency_stop
    uint8_t  control_mode;      // 0: info only, 1: speed control, 2: gap control
    uint8_t  reserved0;
    uint8_t  reserved1;

    float target_speed_mps;
    float target_gap_m;
    float target_rel_x_m;
    float target_rel_y_m;

    float leader_speed_mps;
    float leader_accel_mps2;

    uint32_t timestamp_ms;
    uint16_t seq;
} control_command_packet_t;
```

### 필드 설명

| 필드 | 설명 |
|---|---|
| platoon_id | 플레툰 ID |
| target_vehicle_id | 명령 대상 차량 |
| command_type | 명령 종류 |
| control_mode | 제어 모드 |
| target_speed_mps | 목표 속도 |
| target_gap_m | 목표 간격 |
| target_rel_x_m | 목표 상대 X 위치 |
| target_rel_y_m | 목표 상대 Y 위치 |
| leader_speed_mps | leader 현재 속도 |
| leader_accel_mps2 | leader 현재 가속도 |
| timestamp_ms | 데이터 생성 시각 |
| seq | 패킷 순서 번호 |

---

## 8. Emergency Packet

### 목적

Emergency Packet은 급정지, 충돌 위험, UWB 신뢰도 상실, ESP-NOW 통신 두절 등의 안전 상황을 알리는 패킷이다.

가장 높은 우선순위를 가진다.

### 송신 주기

```text
이벤트 발생 즉시 송신
필요 시 짧은 주기로 반복 송신
```

### Payload 구조체

```c
typedef struct {
    uint32_t vehicle_id;
    uint32_t platoon_id;

    uint8_t emergency_type;
    uint8_t severity;           // 0~255
    uint8_t source;             // 0: ESP, 1: Raspberry Pi, 2: UWB, 3: manual
    uint8_t reserved0;

    float current_speed_mps;
    float distance_to_front_m;
    float relative_speed_mps;

    uint32_t error_code;
    uint32_t timestamp_ms;
    uint16_t seq;
} emergency_packet_t;
```

### Emergency type 정의

```c
#define EMG_NONE              0x00
#define EMG_STOP              0x01
#define EMG_COLLISION_RISK    0x02
#define EMG_UWB_LOST          0x03
#define EMG_ESPNOW_LOST       0x04
#define EMG_PLATOON_BREAK     0x05
```

### 필드 설명

| 필드 | 설명 |
|---|---|
| vehicle_id | 긴급 상황을 발생시킨 차량 ID |
| platoon_id | 대상 플레툰 ID |
| emergency_type | 긴급 상황 종류 |
| severity | 위험도 |
| source | 긴급 상황 발생 출처 |
| current_speed_mps | 현재 차량 속도 |
| distance_to_front_m | 앞 차량까지 거리 |
| relative_speed_mps | 앞차와의 상대 속도 |
| error_code | 상세 오류 코드 |
| timestamp_ms | 데이터 생성 시각 |
| seq | 패킷 순서 번호 |

---

## 9. ACK Packet

### 목적

ACK Packet은 특정 패킷을 정상적으로 받았는지 확인하기 위한 응답 패킷이다.

ESP-NOW broadcast는 기본적으로 모든 수신 차량에 대한 신뢰성 확인이 어렵기 때문에, 중요한 명령이나 긴급 패킷에 대해서만 ACK를 사용하는 것이 좋다.

### Payload 구조체

```c
typedef struct {
    uint32_t acked_vehicle_id;
    uint16_t acked_seq;
    uint8_t  acked_msg_type;
    uint8_t  status;            // 0: OK, 1: error, 2: duplicated, 3: outdated

    uint32_t timestamp_ms;
} ack_packet_t;
```

---

## 10. UWB AoA 측정 결과와 ESP-NOW 패킷 매칭

UWB AoA 측정 결과는 다음과 같은 형태로 얻어진다.

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

ESP-NOW Vehicle Status Packet을 통해 주변 차량의 `vehicle_id ↔ uwb_id` 관계를 알고 있으면, UWB 결과를 차량 ID와 매칭할 수 있다.

```text
UWB result:
target_uwb_id = 0xA002
distance = 3.42 m
angle = -7.5 deg

Vehicle table:
vehicle_id = 102
uwb_id = 0xA002

매칭 결과:
vehicle_id 102는 내 기준 거리 3.42 m, 방위각 -7.5도에 있다.
```

상대 좌표는 다음과 같이 계산할 수 있다.

```text
rel_x = distance × cos(angle)
rel_y = distance × sin(angle)
```

좌표계 정의는 다음과 같이 잡는다.

```text
차량 진행 방향 = +X
차량 왼쪽 = +Y
차량 오른쪽 = -Y
차량 뒤쪽 = -X
```

---

## 11. ESP32 내부 Vehicle Table

ESP32는 ESP-NOW 정보와 UWB 정보를 합쳐서 주변 차량 테이블을 유지한다.

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

이 테이블은 Raspberry Pi로 전달할 최종 데이터의 기반이 된다.

---

## 12. Raspberry Pi로 전달하는 최종 데이터

ESP32는 최종적으로 Raspberry Pi에 다음과 같은 형태의 데이터를 보낸다.

초기 개발에서는 JSON을 추천한다.

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
      "platoon_role": 2,
      "platoon_index": 1,
      "confidence": 0.91
    }
  ]
}
```

라즈베리파이에서는 이 데이터를 ROS topic으로 변환할 수 있다.

```text
/uwb/targets
/platoon/vehicles
/platoon/control_cmd
/emergency
```

---

## 13. 추천 구현 순서

### 1단계: Vehicle Status Packet

```text
각 차량이 자신의 vehicle_id, uwb_id, speed, platoon 상태를 broadcast
```

### 2단계: Vehicle Table 생성

```text
수신한 Vehicle Status Packet으로 vehicle_id ↔ uwb_id 테이블 생성
```

### 3단계: Platoon Info Packet

```text
leader가 platoon_id, member list, target gap, target speed broadcast
```

### 4단계: UWB AoA 매칭

```text
UWB result의 target_uwb_id를 vehicle table과 매칭
```

### 5단계: Raspberry Pi 전송

```text
vehicle_id별 distance, angle, rel_x, rel_y, speed, platoon 정보를 Pi로 전송
```

### 6단계: TDMA Sync 추가

```text
차량이 3대 이상으로 늘어나면 TDMA slot을 적용
```

### 7단계: Control / Emergency 패킷 추가

```text
leader 제어 명령과 긴급 상황 처리를 추가
```

---

## 14. 패킷별 추천 송신 주기

| 패킷 | 송신 주기 | 우선순위 | 초기 구현 |
|---|---:|---:|---|
| Vehicle Status | 50~100 ms | 중간 | 필수 |
| Platoon Info | 100~500 ms | 중간 | 필수 |
| TDMA Sync | 100~500 ms | 높음 | 2단계 |
| Control Command | 20~100 ms | 높음 | 3단계 |
| Emergency | 발생 즉시, 반복 송신 | 최상 | 4단계 |
| ACK | 필요 시 | 낮음 | 선택 |

---

## 15. 핵심 필드 요약

초기 구현에서 반드시 필요한 필드는 다음과 같다.

```text
vehicle_id
uwb_id
platoon_id
platoon_enable
platoon_role
platoon_index
speed_mps
heading_deg
timestamp_ms
seq
```

UWB AoA 측정값과 매칭한 뒤에는 다음 필드가 추가된다.

```text
distance_m
angle_deg
rel_x_m
rel_y_m
confidence
```

최종적으로 Raspberry Pi는 다음 정보를 사용한다.

```text
앞차 ID
앞차 거리
앞차 방위각
앞차 상대 x
앞차 상대 y
앞차 속도
플레툰 목표 간격
confidence
긴급 상태
```

---

## 16. 최종 정리

본 프로젝트의 차량 간 패킷 구조는 다음과 같이 정리한다.

```text
ESP-NOW 패킷:
- Vehicle Status Packet
- Platoon Info Packet
- TDMA Sync Packet
- Control Command Packet
- Emergency Packet
- ACK Packet

UWB AoA 결과:
- target_uwb_id
- distance_m
- angle_deg
- signal quality

ESP32 내부:
- UWB_ID와 vehicle_id를 매칭
- distance/angle을 차량 정보와 결합
- vehicle table 유지

Raspberry Pi:
- ESP32에서 받은 vehicle table을 ROS topic으로 변환
- 플레툰 제어 및 자율주행 판단 수행
```

가장 중요한 설계 기준은 다음과 같다.

```text
ESP-NOW는 차량의 의미 정보를 공유한다.
UWB AoA는 차량의 물리 위치 정보를 측정한다.
ESP32는 UWB_ID를 기준으로 두 정보를 매칭한다.
Raspberry Pi는 매칭된 상대 차량 정보를 사용해 플레툰 제어를 수행한다.
```
