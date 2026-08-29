# Raspberry Pi 플래툰 알고리즘 연동용 ESP32-S3 V2X 인터페이스 정리

## 1. 목적

본 문서는 Raspberry Pi에서 플래툰 알고리즘을 개발하기 위해, ESP32-S3와 주고받아야 하는 데이터를 정리한 문서이다.

본 프로젝트의 기본 구조는 다음과 같다.

```text
ESP-NOW:
- 차량 ID
- UWB ID
- 속도
- heading
- 플래툰 ID
- 플래툰 역할
- 플래툰 순서
- 제어 명령
- 긴급 상태

UWB AoA:
- target UWB ID
- 거리 distance
- 방위각 angle
- 수신 품질
```

핵심 개념은 다음과 같다.

```text
ESP-NOW로 “이 차량이 누구인지” 공유하고,
UWB AoA로 “그 차량이 어디에 있는지” 측정한다.
두 정보는 UWB_ID를 기준으로 매칭한다.
```

즉, ESP32-S3는 V2V 센서/통신 게이트웨이이고, Raspberry Pi는 플래툰 판단 및 제어 알고리즘을 수행하는 상위 제어기이다.

---

## 2. 전체 시스템 역할 분리

### 2.1 ESP32-S3 역할

ESP32-S3는 다음 역할을 담당한다.

```text
1. DW3000 UWB AoA 보드 제어
2. UWB를 통한 거리, 각도, UWB_ID 수집
3. ESP-NOW를 통한 차량 상태 정보 송수신
4. UWB_ID와 vehicle_id 매칭
5. TDMA 기반 UWB ranging 시간 관리
6. 주변 차량 테이블 유지
7. Raspberry Pi로 최종 상대 차량 정보 전달
```

즉 ESP32-S3는 UWB에서 얻은 물리 위치 정보와 ESP-NOW에서 받은 차량 의미 정보를 합쳐 Raspberry Pi로 전달한다.

### 2.2 Raspberry Pi 역할

Raspberry Pi는 다음 역할을 담당한다.

```text
1. ESP32-S3로부터 주변 차량 정보 수신
2. 상대 차량 위치 정보 파싱
3. 상대 위치 필터링
4. 플래툰 합류/유지/이탈 판단
5. 목표 속도 및 목표 차간거리 결정
6. STM32 또는 하위 제어기로 주행 제어 명령 전달
7. 로그 저장 및 시각화
8. 필요 시 ROS2 topic 변환
```

기존 구조에서는 ESP32의 `vehicle_table`이 `pi_serial_bridge`를 통해 Raspberry Pi로 전달되고, Raspberry Pi에서는 ROS2 serial node를 통해 `/uwb/targets`, `/platoon/vehicles`, `/platoon/control_cmd` 등의 토픽으로 변환하는 구조를 고려한다.

---

## 3. ESP32-S3 ↔ Raspberry Pi 연결 방식

ESP32-S3와 Raspberry Pi는 USB CDC 기반 Serial 통신으로 연결한다.

```text
Raspberry Pi USB 포트  ↔  ESP32-S3 USB 포트
/dev/ttyACM0 예상       USB CDC Serial
```

초기 통신 형식은 디버깅이 쉬운 JSON Line 방식을 사용한다.

```text
메시지 1개 = JSON 문자열 1개
메시지 끝 = '\n'
```

예시는 다음과 같다.

```json
{"type":"self_status","vehicle_id":101,"speed_mps":0.42}
```

실제 전송 시에는 마지막에 줄바꿈을 붙인다.

```text
{"type":"self_status","vehicle_id":101,"speed_mps":0.42}\n
```

---

## 4. USB CDC 통신 패킷 개요

초기 구현에서 필요한 패킷은 2개이다.

```text
Raspberry Pi → ESP32-S3 : self_status
ESP32-S3 → Raspberry Pi : v2x_targets
```

### 4.1 Pi → ESP32-S3: `self_status`

Raspberry Pi가 ESP32-S3에게 **내 차량의 현재 상태**를 알려주는 패킷이다.

ESP32-S3는 이 정보를 받아서 ESP-NOW Vehicle Status Packet에 반영하고, 주변 차량으로 broadcast한다.

### 4.2 ESP32-S3 → Pi: `v2x_targets`

ESP32-S3가 Raspberry Pi에게 **주변 차량 정보 테이블**을 전달하는 패킷이다.

ESP32-S3는 ESP-NOW로 받은 차량 상태 정보와 UWB로 측정한 거리/각도 정보를 `UWB_ID` 기준으로 매칭한 뒤 Raspberry Pi로 전달한다.

---

## 5. Pi → ESP32-S3 패킷: `self_status`

### 5.1 목적

`self_status`는 Raspberry Pi가 ESP32-S3에게 내 차량의 상태를 전달하는 패킷이다.

이 패킷에는 단순히 속도만 들어가는 것이 아니라, 현재 차량이 플래툰에 참여 중인지, 어떤 플래툰인지, 리더인지 팔로워인지, 대열에서 몇 번째인지, 앞차가 누구인지 등의 정보가 포함되어야 한다.

ESP32-S3는 이 정보를 ESP-NOW 차량 상태 broadcast에 반영한다.

기존 Vehicle Status Packet에는 `vehicle_id`, `uwb_id`, `platoon_id`, `platoon_enable`, `platoon_role`, `platoon_index`, `driving_state`, `speed_mps`, `heading_deg`, `timestamp_ms`, `seq`가 포함되도록 정의되어 있다.

---

### 5.2 `self_status` 필수 필드

```text
type
seq
timestamp_ms

vehicle_id
uwb_id
destination_id

driving_state
platoon_state

speed_mps
heading_deg

platoon_enable
platoon_id
platoon_role
platoon_index

leader_vehicle_id
front_vehicle_id

target_speed_mps
target_gap_m
```

---

### 5.3 `self_status` JSON 형식

```json
{
  "type": "self_status",
  "seq": 1,
  "timestamp_ms": 12345,

  "vehicle_id": 101,
  "uwb_id": 40961,
  "destination_id": 3,

  "driving_state": 3,
  "platoon_state": 3,

  "speed_mps": 0.42,
  "heading_deg": 0.0,

  "platoon_enable": 1,
  "platoon_id": 7,
  "platoon_role": 2,
  "platoon_index": 1,

  "leader_vehicle_id": 101,
  "front_vehicle_id": 101,

  "target_speed_mps": 0.50,
  "target_gap_m": 1.50
}
```

---

### 5.4 `self_status` 한 줄 전송 예시

USB CDC로 실제 전송할 때는 한 줄 JSON으로 보낸다.

```text
{"type":"self_status","seq":1,"timestamp_ms":12345,"vehicle_id":101,"uwb_id":40961,"destination_id":3,"driving_state":3,"platoon_state":3,"speed_mps":0.42,"heading_deg":0.0,"platoon_enable":1,"platoon_id":7,"platoon_role":2,"platoon_index":1,"leader_vehicle_id":101,"front_vehicle_id":101,"target_speed_mps":0.50,"target_gap_m":1.50}
```

마지막에 반드시 `\n`을 붙인다.

```text
JSON + '\n'
```

---

### 5.5 `self_status` 필드 설명

| 필드 | 설명 |
|---|---|
| `type` | 메시지 종류. `"self_status"` 고정 |
| `seq` | Raspberry Pi가 증가시키는 패킷 번호 |
| `timestamp_ms` | Raspberry Pi 기준 송신 시간 |
| `vehicle_id` | 내 차량 ID |
| `uwb_id` | 내 차량의 UWB ID |
| `destination_id` | 목적지 ID |
| `driving_state` | 주행 모드 |
| `platoon_state` | 플래툰 FSM 상태 |
| `speed_mps` | 현재 차량 속도 |
| `heading_deg` | 현재 차량 진행 방향 |
| `platoon_enable` | 플래툰 참여 여부 |
| `platoon_id` | 현재 소속 플래툰 ID |
| `platoon_role` | 플래툰 내 역할 |
| `platoon_index` | 플래툰 내 순서 |
| `leader_vehicle_id` | 리더 차량 ID |
| `front_vehicle_id` | 내 바로 앞 차량 ID |
| `target_speed_mps` | 플래툰 목표 속도 |
| `target_gap_m` | 목표 차간 거리 |

---

## 6. `self_status` 상황별 예시

### 6.1 플래툰이 아닌 일반 자율주행 상태

```json
{
  "type": "self_status",
  "seq": 1,
  "timestamp_ms": 10000,

  "vehicle_id": 101,
  "uwb_id": 40961,
  "destination_id": 3,

  "driving_state": 2,
  "platoon_state": 1,

  "speed_mps": 0.42,
  "heading_deg": 0.0,

  "platoon_enable": 0,
  "platoon_id": 0,
  "platoon_role": 0,
  "platoon_index": 0,

  "leader_vehicle_id": 0,
  "front_vehicle_id": 0,

  "target_speed_mps": 0.0,
  "target_gap_m": 0.0
}
```

의미:

```text
나는 101번 차량이고,
UWB ID는 0xA001이며,
현재 AUTO/SOLO 상태이고,
플래툰에는 참여하지 않았다.
```

---

### 6.2 플래툰 리더 차량

```json
{
  "type": "self_status",
  "seq": 2,
  "timestamp_ms": 12000,

  "vehicle_id": 101,
  "uwb_id": 40961,
  "destination_id": 3,

  "driving_state": 3,
  "platoon_state": 3,

  "speed_mps": 0.50,
  "heading_deg": 0.0,

  "platoon_enable": 1,
  "platoon_id": 7,
  "platoon_role": 1,
  "platoon_index": 0,

  "leader_vehicle_id": 101,
  "front_vehicle_id": 0,

  "target_speed_mps": 0.50,
  "target_gap_m": 1.50
}
```

의미:

```text
나는 101번 차량이고,
플래툰 7번의 리더이며,
대열 순서는 0번이다.
내 앞차는 없다.
목표 속도는 0.50 m/s이다.
```

---

### 6.3 플래툰 follower 차량

```json
{
  "type": "self_status",
  "seq": 3,
  "timestamp_ms": 13000,

  "vehicle_id": 102,
  "uwb_id": 40962,
  "destination_id": 3,

  "driving_state": 3,
  "platoon_state": 3,

  "speed_mps": 0.48,
  "heading_deg": 0.0,

  "platoon_enable": 1,
  "platoon_id": 7,
  "platoon_role": 2,
  "platoon_index": 1,

  "leader_vehicle_id": 101,
  "front_vehicle_id": 101,

  "target_speed_mps": 0.50,
  "target_gap_m": 1.50
}
```

의미:

```text
나는 102번 차량이고,
플래툰 7번에 follower로 참여 중이며,
대열 순서는 1번이다.
리더 차량은 101번이고,
내 바로 앞차도 101번이다.
목표 차간 거리는 1.5 m이다.
```

---

## 7. ESP32-S3 → Pi 패킷: `v2x_targets`

### 7.1 목적

`v2x_targets`는 ESP32-S3가 Raspberry Pi에게 주변 차량 정보를 전달하는 패킷이다.

ESP32-S3 내부의 Vehicle Table은 ESP-NOW로 받은 정보와 UWB로 측정한 정보를 결합한다.

```text
ESP-NOW로 받은 정보:
vehicle_id, uwb_id, speed, heading, platoon 상태

UWB로 측정한 정보:
uwb_id, distance, angle, signal quality
```

이 두 정보는 `uwb_id` 기준으로 매칭된다.

---

### 7.2 `v2x_targets` JSON 형식

```json
{
  "type": "v2x_targets",
  "seq": 10,
  "timestamp_ms": 22345,
  "self_vehicle_id": 101,

  "targets": [
    {
      "vehicle_id": 102,
      "uwb_id": 40962,

      "distance_m": 3.42,
      "angle_deg": -7.5,
      "rel_x_m": 3.39,
      "rel_y_m": -0.45,

      "speed_mps": 1.20,
      "heading_deg": 3.5,

      "driving_state": 3,
      "platoon_state": 3,
      "platoon_id": 7,
      "platoon_enable": 1,
      "platoon_role": 2,
      "platoon_index": 1,

      "uwb_valid": 1,
      "espnow_valid": 1,
      "confidence": 0.91
    }
  ]
}
```

---

### 7.3 `v2x_targets` 한 줄 전송 예시

```text
{"type":"v2x_targets","seq":10,"timestamp_ms":22345,"self_vehicle_id":101,"targets":[{"vehicle_id":102,"uwb_id":40962,"distance_m":3.42,"angle_deg":-7.5,"rel_x_m":3.39,"rel_y_m":-0.45,"speed_mps":1.20,"heading_deg":3.5,"driving_state":3,"platoon_state":3,"platoon_id":7,"platoon_enable":1,"platoon_role":2,"platoon_index":1,"uwb_valid":1,"espnow_valid":1,"confidence":0.91}]}
```

마지막에 반드시 `\n`을 붙인다.

문서의 Raspberry Pi 전달 예시도 `self_vehicle_id`, `timestamp_ms`, `targets` 배열 안에 `vehicle_id`, `uwb_id`, `distance_m`, `angle_deg`, `rel_x_m`, `rel_y_m`, `speed_mps`, `platoon_id`, `platoon_role`, `platoon_index`, `confidence`를 포함하는 구조이다.

---

### 7.4 `v2x_targets` 필드 설명

| 필드 | 설명 |
|---|---|
| `type` | 메시지 종류. `"v2x_targets"` 고정 |
| `seq` | ESP32-S3가 증가시키는 패킷 번호 |
| `timestamp_ms` | ESP32-S3 기준 송신 시간 |
| `self_vehicle_id` | 현재 ESP32-S3가 속한 차량 ID |
| `targets` | 주변 차량 목록 배열 |
| `vehicle_id` | 상대 차량 ID |
| `uwb_id` | 상대 차량 UWB ID |
| `distance_m` | 상대 차량까지 거리 |
| `angle_deg` | 상대 차량 방위각 |
| `rel_x_m` | 전후 상대 위치 |
| `rel_y_m` | 좌우 상대 위치 |
| `speed_mps` | 상대 차량 속도 |
| `heading_deg` | 상대 차량 진행 방향 |
| `driving_state` | 상대 차량 주행 모드 |
| `platoon_state` | 상대 차량 플래툰 FSM 상태 |
| `platoon_id` | 상대 차량 플래툰 ID |
| `platoon_enable` | 상대 차량 플래툰 참여 여부 |
| `platoon_role` | 상대 차량 역할 |
| `platoon_index` | 상대 차량 대열 순서 |
| `uwb_valid` | UWB 측정값 유효 여부 |
| `espnow_valid` | ESP-NOW 수신 정보 유효 여부 |
| `confidence` | 통합 신뢰도 |

---

## 8. 좌표계 정의

라즈베리파이 알고리즘에서는 ESP32-S3가 보내는 `rel_x_m`, `rel_y_m`의 기준을 동일하게 사용해야 한다.

문서 기준 좌표계는 다음과 같다.

```text
차량 진행 방향 = +X
차량 왼쪽     = +Y
차량 오른쪽   = -Y
차량 뒤쪽     = -X
```

각도 정의는 다음과 같다.

```text
θ = 0°    : 정면
θ > 0°   : 왼쪽
θ < 0°   : 오른쪽
θ = ±90° : 측면
```

상대 좌표 계산식은 다음과 같다.

```text
rel_x = distance × cos(theta)
rel_y = distance × sin(theta)
```

이 좌표계와 계산식은 기존 문서에 명시되어 있다.

---

## 9. 상태값 정의

### 9.1 `driving_state`

```text
0: STOP
1: MANUAL
2: AUTO
3: PLATOON
```

### 9.2 `platoon_state`

```text
0: WAIT
1: SOLO
2: JOIN
3: KEEP
4: EXIT
5: PARKING
```

### 9.3 `platoon_role`

```text
0: NONE
1: LEADER
2: FOLLOWER
```

---

## 10. 권장 통신 주기

ESP-NOW Vehicle Status Packet의 권장 송신 주기는 50~100 ms이다.

USB CDC 통신도 주행 통합 단계에서는 비슷한 수준으로 맞추는 것이 좋다.

| 방향 | 패킷 | 디버깅 단계 | 주행 통합 단계 |
|---|---|---:|---:|
| Pi → ESP32-S3 | `self_status` | 500 ms | 50~100 ms |
| ESP32-S3 → Pi | `v2x_targets` | 500 ms | 50~100 ms |
| ESP32-S3 → Pi | `esp_status` | 1000 ms | 1000 ms |

초기 디버깅에서는 500 ms로 시작하고, 통합 주행 단계에서 100 ms 이하로 줄이는 것을 권장한다.

---

## 11. Raspberry Pi 개발자가 우선 구현할 내용

Raspberry Pi 쪽에서는 다음 기능을 우선 구현하면 된다.

```text
1. USB CDC 포트 열기
   - 예상 포트: /dev/ttyACM0

2. JSON Line 단위로 수신
   - readline() 기준
   - 메시지 끝은 '\n'

3. type == "v2x_targets" 메시지 파싱

4. targets 배열에서 주변 차량 정보 추출

5. 각 target에 대해 다음 값 사용
   - vehicle_id
   - distance_m
   - angle_deg
   - rel_x_m
   - rel_y_m
   - speed_mps
   - platoon_state
   - platoon_role
   - confidence

6. 플래툰 알고리즘에서 앞차 선택
   - front_vehicle_id 또는 platoon_index 기준
   - rel_x_m, rel_y_m 기준 보정 가능

7. 주기적으로 self_status JSON 송신
   - 현재 내 차량 상태
   - 현재 속도
   - 플래툰 참여 상태
   - 리더/팔로워 역할
   - 앞차 ID
   - 목표 속도
   - 목표 차간거리
```

---

## 12. Raspberry Pi에서 플래툰 알고리즘이 주로 사용할 값

Raspberry Pi의 플래툰 알고리즘에서는 `v2x_targets`에서 다음 값을 주로 사용하면 된다.

```text
앞차 vehicle_id
앞차 distance_m
앞차 angle_deg
앞차 rel_x_m
앞차 rel_y_m
앞차 speed_mps
앞차 platoon_state
앞차 platoon_role
앞차 platoon_index
confidence
uwb_valid
espnow_valid
```

플래툰 제어에 필요한 핵심 값은 기존 문서에서도 다음과 같이 정리되어 있다.

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

간격 제어와 횡방향 정렬 오차는 다음과 같이 계산할 수 있다.

```text
e_d = target_distance - rel_x
e_y = rel_y
```

해석 예시는 다음과 같다.

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

## 13. 나중에 추가할 수 있는 패킷

초기 구현에서는 `self_status`와 `v2x_targets`만 사용한다.

이후 필요하면 다음 패킷을 추가한다.

### 13.1 `esp_status`

ESP32-S3 상태 보고용 패킷이다.

```json
{
  "type": "esp_status",
  "seq": 50,
  "timestamp_ms": 30000,

  "vehicle_id": 101,
  "espnow_ok": 1,
  "uwb_ok": 1,
  "uwb_mode": "pdoa_test",
  "target_count": 1,
  "free_heap": 185432
}
```

### 13.2 `platoon_cmd`

Raspberry Pi가 ESP32-S3에게 플래툰 명령을 전달할 때 사용할 수 있다.

```json
{
  "type": "platoon_cmd",
  "seq": 20,
  "timestamp_ms": 30000,

  "cmd": "set_keep",
  "platoon_id": 7,
  "target_vehicle_id": 102,
  "leader_vehicle_id": 101,
  "front_vehicle_id": 101,
  "target_speed_mps": 1.0,
  "target_gap_m": 2.0,
  "platoon_role": 2,
  "platoon_index": 1
}
```

### 13.3 `emergency`

긴급 상황 전달용 패킷이다.

```json
{
  "type": "emergency",
  "seq": 30,
  "timestamp_ms": 31000,

  "vehicle_id": 101,
  "platoon_id": 7,
  "emergency_type": 2,
  "severity": 200,
  "source": 1,
  "current_speed_mps": 0.5,
  "distance_to_front_m": 0.4,
  "relative_speed_mps": -0.2,
  "error_code": 0
}
```

Emergency Packet은 급정지, 충돌 위험, UWB 신뢰도 상실, ESP-NOW 통신 두절 등의 안전 상황을 알리는 최우선 패킷으로 정리되어 있다.

---

## 14. 최종 요약

### 14.1 물리 연결

```text
Raspberry Pi ↔ ESP32-S3 USB CDC Serial
예상 포트: /dev/ttyACM0
```

### 14.2 데이터 형식

```text
JSON Line
메시지 1개 = JSON 문자열 1개
메시지 끝 = '\n'
```

### 14.3 Pi → ESP32-S3

```text
패킷 이름:
self_status

의미:
내 차량의 ID, UWB ID, 주행 상태, 속도, 목적지,
플래툰 참여 여부, 플래툰 ID, 역할, 대열 순서,
리더 차량 ID, 앞차 ID, 목표 속도, 목표 차간거리 전달
```

### 14.4 ESP32-S3 → Pi

```text
패킷 이름:
v2x_targets

의미:
ESP-NOW 의미 정보와 UWB 물리 위치 정보를 UWB_ID 기준으로 매칭한
주변 차량 테이블 전달
```

### 14.5 Raspberry Pi 개발자가 먼저 구현할 것

```text
1. /dev/ttyACM0에서 JSON Line 수신
2. v2x_targets 파싱
3. targets 배열에서 앞차 정보 선택
4. distance_m, angle_deg, rel_x_m, rel_y_m, speed_mps, confidence 사용
5. 플래툰 제어 알고리즘 수행
6. self_status를 주기적으로 ESP32-S3로 송신
```