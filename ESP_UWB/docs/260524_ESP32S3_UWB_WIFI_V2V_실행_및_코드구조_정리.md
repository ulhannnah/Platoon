# ESP32-S3 UWB/Wi-Fi V2V 게이트웨이 실행 및 코드 구조 정리

## 1. 현재 구현 상태

현재 `src` 폴더에는 ESP-IDF 기반 ESP32-S3 펌웨어 프로젝트가 들어 있다.

이 코드는 원래 DW3000 UWB 보드를 붙여서 거리와 각도를 측정하는 구조를 목표로 하지만, 현재 단계에서는 UWB 연결이 어려울 수 있으므로 **UWB는 인터페이스만 준비해 두고 ESP-NOW 기반 Wi-Fi 통신만으로도 차량 간 연결을 확인할 수 있게** 구성했다.

현재 확인된 동작은 다음과 같다.

```text
보드 101 -> 보드 102 수신 성공
보드 102 -> 보드 101 수신 성공
ESP-NOW 양방향 통신 성공
Serial JSON 출력 성공
```

UWB가 아직 연결되지 않은 상태에서는 거리와 각도 값이 0으로 출력되는 것이 정상이다.

```json
{
  "self_vehicle_id": 101,
  "targets": [
    {
      "vehicle_id": 102,
      "uwb_id": 40962,
      "distance_m": 0.00,
      "angle_deg": 0.00,
      "rel_x_m": 0.00,
      "rel_y_m": 0.00,
      "confidence": 0.50
    }
  ]
}
```

---

## 2. 폴더 구조

프로젝트 위치는 다음과 같다.

```text
D:\WorkSpace\Platoon\ESP_UWB\src
```

주요 구조는 다음과 같다.

```text
src/
├── CMakeLists.txt
├── sdkconfig.defaults
├── README.md
└── main/
    ├── CMakeLists.txt
    ├── main.c
    ├── app_config.h
    ├── packet_defs.h
    ├── vehicle_table.c
    ├── vehicle_table.h
    ├── uwb_manager.c
    ├── uwb_manager.h
    ├── espnow_manager.c
    ├── espnow_manager.h
    ├── tdma_scheduler.c
    ├── tdma_scheduler.h
    ├── id_matcher.c
    ├── id_matcher.h
    ├── pi_serial_bridge.c
    ├── pi_serial_bridge.h
    ├── system_monitor.c
    └── system_monitor.h
```

---

## 3. 코드 모듈 역할

## 3.1 `main.c`

전체 FreeRTOS task를 생성하고 시스템을 시작한다.

주요 task는 다음과 같다.

```text
task_espnow_tx
- 내 차량 상태를 ESP-NOW broadcast로 주기적 송신

task_uwb
- UWB 사용 가능 여부 확인
- UWB mock 또는 실제 UWB 결과가 있으면 vehicle_table에 반영

task_pi_tx
- vehicle_table 내용을 JSON으로 serial 출력

task_monitor
- 오래된 차량 정보 timeout 처리
```

## 3.2 `app_config.h`

차량 ID, UWB ID, 플레툰 번호, ESP-NOW 채널, 주기 설정을 모아둔 파일이다.

보드마다 반드시 다르게 설정해야 하는 값:

```c
#define APP_SELF_VEHICLE_ID
#define APP_SELF_UWB_ID
#define APP_PLATOON_INDEX
```

두 보드가 같게 유지해야 하는 값:

```c
#define APP_PLATOON_ID
#define APP_ESPNOW_CHANNEL
```

## 3.3 `espnow_manager.c`

ESP-NOW 통신을 담당한다.

동작:

```text
1. NVS 초기화
2. Wi-Fi STA 모드 시작
3. ESP-NOW 초기화
4. broadcast peer 등록
5. 내 vehicle status packet 송신
6. 다른 보드 packet 수신 시 vehicle_table 갱신
```

현재는 AP 공유기에 접속하는 일반 Wi-Fi 방식이 아니라, ESP32끼리 직접 통신하는 ESP-NOW 방식이다.

## 3.4 `vehicle_table.c`

주변 차량 정보를 저장하는 핵심 테이블이다.

ESP-NOW로 받은 정보:

```text
vehicle_id
uwb_id
speed_mps
heading_deg
platoon_id
platoon_role
platoon_index
```

UWB로 받을 예정인 정보:

```text
distance_m
angle_deg
rel_x_m
rel_y_m
```

현재 UWB가 없으면 거리/각도는 0으로 남고, ESP-NOW 정보만 채워진다.

## 3.5 `uwb_manager.c`

DW3000 UWB 보드를 붙일 자리를 미리 만들어 둔 모듈이다.

현재 기본 설정에서는 실제 UWB 드라이버가 없으므로 다음처럼 동작한다.

```text
UWB 초기화 시도
-> DW3000 driver not linked
-> UWB unavailable
-> ESP-NOW only 모드로 계속 실행
```

즉 UWB가 없어도 펌웨어는 죽지 않고 계속 실행된다.

UWB 가짜 데이터를 테스트하려면 `app_config.h`에서 다음 값을 바꾼다.

```c
#define APP_ENABLE_UWB_MOCK        1
```

기본값은 다음과 같다.

```c
#define APP_ENABLE_UWB_MOCK        0
```

## 3.6 `pi_serial_bridge.c`

Raspberry Pi 또는 PC monitor로 보낼 JSON을 만든다.

출력 예:

```json
{"self_vehicle_id":101,"timestamp_ms":58597,"targets":[{"vehicle_id":102,"uwb_id":40962,"distance_m":0.00,"angle_deg":0.00,"rel_x_m":0.00,"rel_y_m":0.00,"speed_mps":0.00,"heading_deg":0.00,"platoon_id":7,"confidence":0.50}]}
```

나중에 Raspberry Pi ROS2 node에서는 이 JSON 한 줄을 읽어서 `/uwb/targets` 또는 `/platoon/vehicles` 토픽으로 변환하면 된다.

---

## 4. ESP-IDF 실행 준비

현재 PC에는 ESP-IDF v5.4가 설치되어 있다.

일반 PowerShell에서 `idf.py`가 인식되지 않으면 먼저 다음 명령을 실행한다.

```powershell
. C:\Espressif\Initialize-Idf.ps1 -IdfId esp-idf-1d48cf7427aa464d60cc013d481736ef
```

주의할 점:

```text
맨 앞의 점(.)이 중요하다.
이 점은 현재 PowerShell 세션에 ESP-IDF 환경을 불러오는 역할을 한다.
```

정상 확인:

```powershell
idf.py --version
```

또는 Windows 시작 메뉴에서 `ESP-IDF 5.4 PowerShell`을 열면 초기화가 자동으로 되어 있을 수 있다.

---

## 5. 빌드 방법

프로젝트 폴더로 이동한다.

```powershell
cd D:\WorkSpace\Platoon\ESP_UWB\src
```

타깃을 ESP32-S3로 설정한다.

```powershell
idf.py set-target esp32s3
```

빌드한다.

```powershell
idf.py build
```

정상 빌드되면 다음과 유사한 메시지가 나온다.

```text
Project build complete.
Generated build/esp32s3_uwb_v2v_gateway.bin
```

---

## 6. 보드 1대 실행 방법

보드 포트를 확인한다.

```powershell
[System.IO.Ports.SerialPort]::GetPortNames()
```

예를 들어 보드가 `COM6`이면 다음처럼 flash와 monitor를 실행한다.

```powershell
idf.py -p COM6 flash monitor
```

정상 실행 시 출력 예:

```text
UWB unavailable, continuing with Wi-Fi/ESP-NOW only
ESP-NOW ready on channel 1
{"self_vehicle_id":101,"timestamp_ms":121017,"targets":[]}
```

`targets:[]`는 아직 상대 보드가 없다는 뜻이다.

---

## 7. 보드 2대 통신 확인 방법

## 7.1 1번 보드 설정

`main/app_config.h`를 다음처럼 설정한다.

```c
#define APP_SELF_VEHICLE_ID        101u
#define APP_SELF_UWB_ID            0xA001u
#define APP_PLATOON_INDEX          0u
```

예를 들어 포트가 `COM6`이면:

```powershell
idf.py -p COM6 flash monitor
```

## 7.2 2번 보드 설정

같은 `main/app_config.h`를 다음처럼 바꾼다.

```c
#define APP_SELF_VEHICLE_ID        102u
#define APP_SELF_UWB_ID            0xA002u
#define APP_PLATOON_INDEX          1u
```

예를 들어 포트가 `COM7`이면:

```powershell
idf.py -p COM7 flash monitor
```

## 7.3 성공 기준

1번 보드 출력에 2번 보드가 보이면 성공이다.

```json
{"self_vehicle_id":101,"targets":[{"vehicle_id":102,"uwb_id":40962}]}
```

2번 보드 출력에 1번 보드가 보이면 성공이다.

```json
{"self_vehicle_id":102,"targets":[{"vehicle_id":101,"uwb_id":40961}]}
```

실제로 현재 테스트에서 양방향 수신이 확인되었다.

```text
101번 보드 targets 안에 102번 차량 표시
102번 보드 targets 안에 101번 차량 표시
```

---

## 8. 현재 테스트 결과 해석

현재 출력:

```json
{"self_vehicle_id":101,"timestamp_ms":58597,"targets":[{"vehicle_id":102,"uwb_id":40962,"distance_m":0.00,"angle_deg":0.00,"rel_x_m":0.00,"rel_y_m":0.00,"speed_mps":0.00,"heading_deg":0.00,"platoon_id":7,"confidence":0.50}]}
```

의미:

```text
self_vehicle_id = 101
- 이 보드는 101번 차량이다.

targets 안의 vehicle_id = 102
- 102번 차량의 ESP-NOW packet을 수신했다.

uwb_id = 40962
- 0xA002를 10진수로 출력한 값이다.

distance_m, angle_deg = 0
- 아직 UWB 측정값이 없으므로 정상이다.

confidence = 0.50
- ESP-NOW 정보만 있고 UWB 정보는 없다는 의미의 낮은 신뢰도이다.
```

---

## 9. 유의사항

## 9.1 두 보드 ID를 반드시 다르게 설정

두 보드가 같은 `APP_SELF_VEHICLE_ID`를 쓰면 서로를 자기 자신으로 판단해서 무시할 수 있다.

```c
if (packet->vehicle_id == APP_SELF_VEHICLE_ID) {
    return;
}
```

따라서 보드마다 다음 값은 다르게 둔다.

```c
APP_SELF_VEHICLE_ID
APP_SELF_UWB_ID
APP_PLATOON_INDEX
```

## 9.2 ESP-NOW 채널은 같아야 함

두 보드 모두 같은 채널이어야 한다.

```c
#define APP_ESPNOW_CHANNEL         1
```

한쪽만 다른 채널이면 서로 packet을 받지 못한다.

## 9.3 UWB 미연결은 에러가 아님

현재는 DW3000 드라이버가 아직 연결되지 않았으므로 다음 메시지가 나와도 정상이다.

```text
UWB unavailable, continuing with Wi-Fi/ESP-NOW only
```

이 메시지는 실패로 멈춘 것이 아니라, Wi-Fi/ESP-NOW only 모드로 계속 실행한다는 뜻이다.

## 9.4 `distance_m`, `angle_deg`가 0인 이유

ESP-NOW는 차량 ID, 속도, heading, platoon 정보만 전달한다.

거리와 각도는 UWB에서 들어와야 하므로, UWB가 없으면 다음 값은 0으로 남는다.

```text
distance_m
angle_deg
rel_x_m
rel_y_m
```

## 9.5 monitor 포트 충돌

이미 어떤 터미널에서 `idf.py monitor`가 포트를 잡고 있으면 다른 터미널에서 같은 COM 포트를 열 수 없다.

증상:

```text
Access to the path 'COM7' is denied.
```

해결:

```text
기존 monitor 터미널에서 Ctrl + ] 로 종료
또는 해당 터미널을 닫기
```

## 9.6 현재 소스는 마지막으로 플래시한 보드 설정을 가진다

예를 들어 2번 보드를 플래시하기 위해 `app_config.h`를 102로 바꿨다면, 현재 소스 파일은 102 설정으로 남아 있다.

다시 1번 보드를 빌드/플래시하려면 101 설정으로 되돌려야 한다.

---

## 10. 다음 개발 단계

추천 순서는 다음과 같다.

```text
1. ESP-NOW packet에 실제 speed_mps, heading_deg 입력 경로 추가
2. UWB mock 모드로 distance/angle 파이프라인 검증
3. DW3000 SPI pin 설정 추가
4. DW3000 driver 연결
5. 실제 UWB ranging 결과를 uwb_result_t로 변환
6. Raspberry Pi ROS2 serial node 작성
```

UWB mock 테스트를 하려면:

```c
#define APP_ENABLE_UWB_MOCK        1
```

로 바꾸고 다시 빌드/플래시한다.

그러면 실제 UWB 없이도 `distance_m`, `angle_deg`, `rel_x_m`, `rel_y_m`가 채워지는지 확인할 수 있다.

---

## 11. 자주 쓰는 명령 모음

ESP-IDF 환경 초기화:

```powershell
. C:\Espressif\Initialize-Idf.ps1 -IdfId esp-idf-1d48cf7427aa464d60cc013d481736ef
```

프로젝트 이동:

```powershell
cd D:\WorkSpace\Platoon\ESP_UWB\src
```

타깃 설정:

```powershell
idf.py set-target esp32s3
```

빌드:

```powershell
idf.py build
```

COM6에 업로드 및 모니터:

```powershell
idf.py -p COM6 flash monitor
```

COM7에 업로드 및 모니터:

```powershell
idf.py -p COM7 flash monitor
```

포트 목록 확인:

```powershell
[System.IO.Ports.SerialPort]::GetPortNames()
```

monitor 종료:

```text
Ctrl + ]
```
