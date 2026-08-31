# [긴급] ESP32-S3 시리얼 출력 충돌로 v2x_targets가 사실상 수신 불가

Raspberry Pi 쪽 V2X 연동 구현(`v2x_node.py`)을 실제 ESP32-S3 보드에 연결해서
테스트하다가 발견한 버그입니다. 인터페이스 문서("Raspberry Pi 플래툰 알고리즘
연동용 ESP32-S3 V2X 인터페이스 정리.md")나 Pi 쪽 구현 문제가 아니라 —
**둘 다 문서 기준으로 정확히 맞게 구현돼 있는 걸 코드 대 코드로 확인했습니다** —
**ESP32 펌웨어 내부의 출력 동시성 버그**입니다. 지금 상태로는 Pi가 `v2x_targets`를
사실상 한 번도 온전히 못 받습니다. 다른 작업(리더 차량 3대 이상 체인, UWB 등)보다
먼저 고쳐야 진행이 가능한 블로커라 우선순위 높게 올려드립니다.

---

## 1. 증상

Pi에서 `v2x_node`(시리얼 수신 노드)를 띄우고 실제 ESP32-S3에 연결해 20초 이상
관찰한 결과, `v2x_targets` JSON을 **단 한 번도 온전하게 파싱하지 못했습니다**
(`ros2 topic hz`로 발행 여부 확인 — 20초 동안 0회).

받은 원본 줄을 그대로 캡처하면 이렇습니다.

```
{"type":"v2x_targets","seq":3001,"timestamp_ms":610486,"self_vehI (826071) espnow_manager: [TX] vehi
```

```
{"type":"v2x_targets","seq":6434,"timestamp_ms":1297186,"self_veI (1575571) espnow_manager: [TX] veh
```

`"self_vehicle_id"` 필드 이름 한가운데(`self_ve` 다음)에 갑자기 **다른 로그
문자열**(`I (826071) espnow_manager: [TX] vehi...`)이 끼어들어 있습니다. 즉 JSON
한 줄을 출력하는 도중에 다른 코드가 같은 시리얼 포트에 동시에 써서, 두 출력이
글자 단위로 섞여버린 것입니다. 이렇게 깨진 줄은 JSON 파싱이 무조건 실패하고,
그 줄이 원래 담고 있던 `v2x_targets` 데이터 전체가 유실됩니다.

`cat /dev/ttyACM0`처럼 Pi가 아무것도 안 보내고 순수하게 읽기만 할 때는 비교적
깨끗하게 나오는 것도 확인했지만, `v2x_node`를 실제로 띄워서 지속적으로 읽는
상황에서는 재현율이 사실상 100%였습니다 (20초 = 약 100회 시도 중 성공 0회).

---

## 2. 원인으로 보이는 코드 위치

같은 콘솔(USB 시리얼)에 서로 다른 FreeRTOS 태스크가 잠금장치 없이 동시에
`printf`/`ESP_LOG`를 호출하고 있습니다.

- `main.c`의 `task_pi_tx` — 200ms 주기로 `pi_serial_bridge_send_snapshot()` 호출
  → 이 안에서 `printf("{\"type\":\"v2x_targets\"...")`로 JSON 한 줄을 씀
- `espnow_manager.c`의 `task_espnow_tx` — 500ms 주기로 `espnow_manager_send_self_status()`
  호출 → 성공 시 `ESP_LOGI(TAG, "[TX] vehicle_id=...")` 로 로그를 씀
- `espnow_manager.c`의 `on_send()` 콜백 — ESP-NOW 전송 완료 시 비동기로
  `printf("[ESP-NOW TX_CB] ...")` 호출
- `espnow_manager.c`의 `on_recv()` 콜백 — 수신 시 여러 줄짜리 `printf` 블록 호출

이 넷 다 서로 다른 타이밍(태스크 주기 200ms/500ms, 콜백은 비동기)에 실행되는데,
콘솔 출력 자체를 보호하는 뮤텍스가 코드 어디에도 없습니다. 200ms와 500ms의
최소공배수가 1초라서 두 주기가 특정 위상으로 한 번 어긋나면 그 상태가 계속
반복되는 것으로 보이고, 그래서 "가끔"이 아니라 "거의 항상" 충돌하는 것 같습니다.

---

## 3. 요청 드리는 수정 방향

아래 둘 중 하나만 적용해도 해결될 것 같습니다 (팀에서 편한 쪽으로 선택).

**옵션 A — 콘솔 출력을 뮤텍스로 보호**

`pi_serial_bridge_send_snapshot()`의 `printf` 블록과, `espnow_manager.c`의
`ESP_LOGI`/`printf` 호출들이 전부 하나의 공유 뮤텍스(`xSemaphoreTake`/`Give`)로
감싸여서 한 번에 하나의 태스크만 콘솔에 쓸 수 있게 만듭니다. `pi_serial_bridge.c`
쪽 스냅샷 출력은 `printf` 여러 번 나눠 호출하는 구조라 하나의 락 안에서 전체를
묶어야 중간에 안 끊깁니다.

**옵션 B — Pi로 가는 포트에서는 디버그 로그를 없애기 (더 간단)**

이 USB 포트는 Pi와의 JSON 프로토콜 전용으로 쓰고, ESP-NOW 디버그 로그
(`[TX]`, `[ESP-NOW TX_CB]`, `on_recv()`의 로그 블록)는 다음 중 하나로 없앱니다.
- `esp_log_level_set("espnow_manager", ESP_LOG_NONE)` 으로 해당 태그 로그 끄기
  (단, `on_send`/`on_recv`의 `printf`는 `ESP_LOG`가 아니라 순수 `printf`라
  로그 레벨로는 안 꺼지니, 이 부분은 삭제하거나 `#ifdef DEBUG`로 감싸야 함)
- 또는 실제 주행/통합 테스트 빌드에서는 이 디버그 `printf`들을 아예 주석 처리

디버깅용 로그가 필요하면 JTAG 콘솔(별도 채널)로 옮기고, USB CDC 쪽은 JSON만
나가게 분리하는 것도 방법입니다.

---

## 4. 참고 — 이 문제와 무관하게 확인된 것들

- 인터페이스 문서와 Pi 구현(`v2x_node.py`)은 필드명, 열거값, 좌표계 계산식까지
  전부 문서 기준과 일치합니다. 이번 버그는 인터페이스 설계 문제가 아닙니다.
- Pi 쪽 파서(`v2x_node.py`)는 깨진 줄을 만나면 조용히 버리고 다음 줄로 넘어가도록
  이미 방어 코드가 있어서, 이 버그 때문에 Pi 쪽이 죽거나 멈추지는 않습니다.
  다만 유효한 데이터를 못 받을 뿐입니다.
- 기존에 보내드린 `esp32_v2x_보강요청.md`(leader_vehicle_id 추가 요청,
  speed_mps 파이프라인 확인 요청)는 이 문제와 별개로 여전히 유효합니다.
