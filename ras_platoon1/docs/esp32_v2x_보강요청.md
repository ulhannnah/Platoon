# ESP32-S3 V2X 패킷 보강 요청

"Raspberry Pi 플래툰 알고리즘 연동용 ESP32-S3 V2X 인터페이스 정리.md" 스펙 기준으로
Raspberry Pi 쪽 구현(v2x_node.py + fsm_decision_node.py 연동)을 마쳤습니다. 그 과정에서
발견한 것 하나를 요청드리고, 확인해주셔야 할 것 하나를 정리했습니다.

---

## 1. 요청 — `v2x_targets.targets[]`에 `leader_vehicle_id` 필드 추가

### 왜 필요한가

플래툰이 리더-팔로워1-팔로워2 순으로 3대 이상 이어질 때, **팔로워2가 팔로워1 뒤에
결합하려면 "팔로워1이 속한 플래툰의 진짜 리더가 누구인지"를 알아야 합니다.** 이걸
모르면 팔로워2가 팔로워1을 리더로 착각해서 잘못된 플래툰 ID로 결합을 시도합니다
(3대 이상 결합에서 흔히 나는 실수 패턴입니다).

### 지금 스펙(§7.2)엔 이 정보가 없음

`targets[]` 각 원소 필드: `vehicle_id`, `uwb_id`, `distance_m`, `angle_deg`, `rel_x_m`,
`rel_y_m`, `speed_mps`, `heading_deg`, `driving_state`, `platoon_state`, `platoon_id`,
`platoon_enable`, `platoon_role`, `platoon_index`, `uwb_valid`, `espnow_valid`, `confidence`

`leader_vehicle_id`는 `self_status`(§5, 내가 보내는 내 정보)에만 있고, **상대 차량
입장에서 "그 상대의 리더가 누구인지"를 알려주는 필드가 없습니다.**

### Pi 쪽에서 임시로 한 것 (완전한 해결책 아님)

같은 `v2x_targets` 패킷 안에 리더 차량도 함께 잡혀 있으면, `platoon_role==1`(LEADER)인
타겟을 찾아서 같은 `platoon_id`를 쓰는 FOLLOWER 타겟들에게 그 `vehicle_id`를 리더로
채워줍니다.

**한계**: 리더가 그 순간 UWB/ESP-NOW 둘 다 안 잡히면(멀거나 가려짐) 여전히 모름
처리됩니다(-1). 3대가 일렬로 늘어서면 맨 뒤 차량 입장에서 리더가 시야 끝/사각에
있을 가능성이 높아서, 이 임시방편만으로는 계속 불안정할 수 있습니다.

### 요청 내용

`targets[]` 각 원소에 `leader_vehicle_id` 필드 추가를 요청드립니다. ESP32의 Vehicle
Table(§6 `tracked_vehicle_t`)에 이미 `platoon_id`/`platoon_role`이 있으니, 같은
`platoon_id`를 쓰는 엔트리 중 `platoon_role==1`인 항목을 찾아 각 타겟에 채워
보내주시면 될 것 같습니다 — ESP32 펌웨어 내부 테이블 순회만으로 가능해 보이고,
UWB/ESP-NOW 실시간 가시성과 무관하게 항상 정확한 값을 줄 수 있습니다.

예시 (필드명만 맞으면 포맷은 자유):

```json
{
  "vehicle_id": 102,
  "platoon_id": 7,
  "platoon_role": 2,
  "leader_vehicle_id": 101
}
```

---

## 2. 확인 요청 — `self_status.speed_mps`가 실제 값으로 나가는지

Pi 쪽 파이프라인을 점검하다가 버그를 하나 발견해서 고쳤습니다. **이전 버전에서는
`self_status.speed_mps`가 항상 0으로 나갔을 겁니다.** 지금은 이렇게 흐릅니다.

```
STM32 (좌우 엔코더 펄스)
    ↓
control_node.py  : (left_delta + right_delta)/2 를 엔코더 CPR·바퀴둘레·경과시간으로
                    m/s로 환산
    ↓ /telemetry (Telemetry.msg에 speed_mps 필드 추가)
fsm_decision_node.py : ego_state.speed에 반영
    ↓ /v2x/self_status (SelfStatus.msg)
v2x_node.py      : 받은 그대로 JSON 직렬화해서 ESP32로 시리얼 송신
```

**확인 부탁드리는 것**: 이전에 테스트하실 때 `self_status.speed_mps`가 항상 0으로
찍혔다면, 이후 버전에서는 0이 아닌 값으로 바뀌는지 재확인해주세요. 엔코더 CPR(1560
pulse/rev)·바퀴지름(65mm)은 아직 실측 전 임시값이라 **절대값 자체는 부정확할 수
있습니다** — 지금 단계에서는 "0 고정이 아니라 차량 속도에 따라 변하는 값인지"만
확인되면 충분합니다.

---

## 3. 포트 관련 참고 (펌웨어와 무관, 우리 쪽 확인 사항)

문서 §3은 ESP32도 `/dev/ttyACM0`을 예상한다고 되어 있는데, 이 차량의 `control_node.py`가
이미 STM32용으로 `/dev/ttyACM0`을 쓰고 있어서 ESP32는 다른 경로(`/dev/ttyACM1` 등)로
잡힐 가능성이 높습니다. 실제 연결 후 저희 쪽에서 `ls /dev/ttyACM*`로 확인해서
맞추겠습니다 — 펌웨어 쪽에서 신경 쓰실 부분은 아닙니다.
