# Platoon Vehicle - STM32F446RE 제어 시스템

STM32F446RE 기반 자율주행 플래툰 시스템의 모터, 서보, 초음파 센서, UART 통신 제어 코드입니다.

---

## 📋 프로젝트 개요

### 주요 하드웨어
- **MCU:** STM32F446RETx (ARM Cortex-M4, 180MHz)
- **타이머:**
  - TIM2: 모터 PWM (1kHz, 1~999 범위)
  - TIM3: 서보 PWM (50Hz, 1000~2000µs)
  - TIM4: 초음파 입력 캡처 (1MHz)
- **통신:** USART2 (115200 baud)

### 핵심 기능
- 🚗 **DC 모터 제어** - PWM + 방향 제어 (전진/후진)
- 🎛️ **서보 제어** - -45°~+45° 범위 (1000~2000µs)
- 📏 **초음파 거리 측정** - cm 단위, 20cm 이내 장애물 감지
- 📡 **Jetson 통신** - 양방향 시리얼 통신

---

## 🏗️ 프로젝트 구조

```
Platoon/
├── Core/
│   ├── Inc/
│   │   ├── main.h
│   │   ├── servo.h              # 서보 제어 API
│   │   ├── encoder.h            # 모터 제어 API
│   │   ├── supersonic.h         # 초음파 센서 API
│   │   ├── serial_communication.h # UART API
│   │   ├── test.h               # ⭐ 테스트 설정
│   │   ├── stm32f4xx_hal_conf.h
│   │   ├── stm32f4xx_it.h
│   │   └── main.h
│   ├── Src/
│   │   ├── main.c
│   │   ├── servo.c              # 서보 구현
│   │   ├── encoder.c            # 모터 구현
│   │   ├── supersonic.c         # 초음파 구현
│   │   ├── serial_communication.c # UART 구현
│   │   ├── test.c               # ⭐ 테스트 루틴
│   │   ├── stm32f4xx_hal_msp.c
│   │   ├── stm32f4xx_it.c
│   │   ├── stm32f4xx_hal_msp.c
│   │   └── ...
│   ├── Startup/
│   │   └── startup_stm32f446retx.s
│   └── ...
├── Drivers/
│   ├── CMSIS/
│   └── STM32F4xx_HAL_Driver/
├── Debug/
├── Platoon.ioc                  # ⭐ STM32CubeIDE 프로젝트
├── STM32F446RETX_FLASH.ld       # 링커 스크립트
└── README.md                    # 이 파일
```

---

## 🚀 빠른 시작

### 1. 프로젝트 열기
```bash
# STM32CubeIDE에서 Platoon.ioc 파일 열기
# 또는 프로젝트 폴더 임포트
```

### 2. 빌드
```
STM32CubeIDE: Build Project (Ctrl+B)
```

### 3. 플래시
```
STM32CubeIDE: Run (Shift+F10) 또는 Debug
```

### 4. 모니터링
```
시리얼 포트 (115200 baud) 연결
예: COM3, /dev/ttyUSB0
```

---

## 🧪 테스트 모드

프로그램 시작 후 각 모듈을 개별 테스트할 수 있습니다.

### 테스트 모드 활성화

#### **Step 1: `Core/Inc/test.h` 열기**

```c
#define TEST_MODE_MOTOR        0   // 모터 테스트
#define TEST_MODE_SERVO        0   // 서보 테스트  
#define TEST_MODE_ULTRASONIC   0   // 초음파 센서 테스트
#define TEST_MODE_SERIAL       0   // UART 통신 테스트
#define TEST_MODE_ALL          0   // 모든 센서 한번에 체크
```

#### **Step 2: 원하는 테스트를 `1`로 변경**

예: 모터 테스트
```c
#define TEST_MODE_MOTOR        1   // ← 이것만 1로 설정
#define TEST_MODE_SERVO        0   
#define TEST_MODE_ULTRASONIC   0   
#define TEST_MODE_SERIAL       0   
#define TEST_MODE_ALL          0   
```

#### **Step 3: 빌드 & 플래시**
```
STM32CubeIDE: Build & Flash
```

#### **Step 4: 시리얼 포트로 결과 확인**

---

## 📝 테스트 모드 상세 설명

### 1️⃣ `TEST_MODE_MOTOR` - 모터 속도/방향 테스트

**동작:**
- 전진: PWM 0 → 500 → 999 (5단계)
- 정지 (1초)
- 후진: PWM 0 → 500 → 999 (5단계)
- 총 약 30초

**예상 출력:**
```
=== MOTOR TEST ===
[FWD] Speed increasing...
PWM: 0 (FWD)
PWM: 100 (FWD)
...
PWM: 900 (FWD)
[STOP] Motor stopped.
[REV] Speed increasing...
PWM: 0 (REV)
...
[END] Motor test complete.
```

**확인 항목:**
- ✅ 모터가 부드럽게 가속/감속되는가?
- ✅ 전진/후진 방향이 맞는가?
- ✅ 정지 명령이 작동하는가?

---

### 2️⃣ `TEST_MODE_SERVO` - 서보 각도 스윕 테스트

**동작:**
- 중앙(0°) 위치 설정
- 좌(-45°) → 우(+45°) → 좌 스윕 (3회 반복)
- 최종 중앙 복귀
- 총 약 45초

**예상 출력:**
```
=== SERVO TEST ===
Center (0 deg)
Angle: 0 deg (pulse: 1500 us)
Angle: -15 deg (pulse: 1333 us)
Angle: -30 deg (pulse: 1167 us)
...
Angle: 45 deg (pulse: 2000 us)
...
[END] Servo test complete. Back to center.
```

**확인 항목:**
- ✅ 서보가 좌/우로 부드럽게 움직이는가?
- ✅ 약 90도 범위(-45°~+45°)로 회전하는가?
- ✅ 펄스값이 1000~2000µs 범위인가?

---

### 3️⃣ `TEST_MODE_ULTRASONIC` - 초음파 센서 거리 측정

**동작:**
- 30초 동안 주기적으로 거리 측정
- 20cm 미만 시 LED(LD2) 점등 + WARNING
- 20cm 이상 시 LED 소등

**예상 출력:**
```
=== ULTRASONIC TEST ===
Measuring for 30 seconds...
Distance: 45.3 cm
Distance: 43.8 cm
Distance: 15.2 cm [WARNING: OBSTACLE NEAR!]
Distance: 5.5 cm [WARNING: OBSTACLE NEAR!]
Distance: 67.2 cm
...
[END] Ultrasonic test complete.
```

**확인 항목:**
- ✅ 센서가 거리를 정확히 측정하는가?
- ✅ 물체를 가까이 가져갔을 때 감지하는가?
- ✅ 20cm 이내 감지 시 LED가 켜지는가?

---

### 4️⃣ `TEST_MODE_SERIAL` - UART 통신 테스트

**동작:**
- 20초 동안 500ms 주기로 모든 센서 데이터 전송
- 외부에서 명령 수신 가능
- 형식: `M:PWM,V:DIR,A:PULSE\n`

**예상 출력:**
```
=== SERIAL TEST ===
Transmitting sensor data every 500ms...
Try sending: M:500,V:1,A:1500
[TX] PWM:0,DIR:2,SERVO:1500,DIST:25.5
[TX] PWM:0,DIR:2,SERVO:1500,DIST:26.1
[TX] PWM:500,DIR:1,SERVO:1500,DIST:25.8
...
[END] Serial test complete.
```

**명령 형식:**
```
M:500,V:1,A:1500\n
```
- `M:500` → 모터 PWM 500 설정
- `V:1` → 전진 (1: 전진, 0: 후진, 2: 정지)
- `A:1500` → 서보 펄스 1500µs (중앙)

**테스트 방법 (터미널/직렬 모니터):**
1. 시리얼 포트 연결 (115200 baud)
2. 다음 명령 입력:
   ```
   M:800,V:1,A:1200
   ```
3. 모터가 PWM 800으로 전진하고 서보가 움직이는지 확인

**확인 항목:**
- ✅ 데이터가 정기적으로 전송되는가?
- ✅ 포맷이 올바른가?
- ✅ 외부 명령을 수신하고 반영하는가?

---

### 5️⃣ `TEST_MODE_ALL` - 모든 센서 동시 작동

**동작:**
- 20초 동안 모든 기능을 동시에 실행
- 모터: 2초 주기로 PWM 변경 (0→250→500→750)
- 서보: 1초 주기로 각도 변경 (-45°~+45° 스윕)
- 초음파: 60ms 주기로 거리 측정
- UART: 1초 주기로 모든 데이터 전송

**예상 출력:**
```
=== COMPREHENSIVE TEST ===
All sensors running simultaneously...
[ALL] M:0,D:2,S:1500,DIST:42.5
[ALL] M:250,D:1,S:1485,DIST:41.2
[ALL] M:250,D:1,S:1470,DIST:40.8
[ALL] M:500,D:0,S:1455,DIST:41.5
...
[END] Comprehensive test complete.
```

**확인 항목:**
- ✅ 모든 기능이 동시에 올바르게 작동하는가?
- ✅ 간섭이나 오류가 없는가?
- ✅ 데이터 전송이 규칙적인가?

---

## 🔄 정상 모드로 복귀

모든 테스트 완료 후, `test.h`의 모든 플래그를 **0으로 설정:**

```c
#define TEST_MODE_MOTOR        0   
#define TEST_MODE_SERVO        0   
#define TEST_MODE_ULTRASONIC   0   
#define TEST_MODE_SERIAL       0   
#define TEST_MODE_ALL          0   
```

그러면 `main()` 루프가 정상 작동:
- 주기적으로 초음파 트리거 (60ms)
- Jetson으로 센서 데이터 전송 (100ms)
- 장애물 20cm 이내 시 긴급 정지

---

## 📡 정상 모드 - Jetson 통신 프로토콜

### STM32 → Jetson (송신, 100ms 주기)
```
PWM:X,DIR:Y,SERVO:Z,DIST:W.W
```
예:
```
PWM:500,DIR:1,SERVO:1500,DIST:45.3
```

### Jetson → STM32 (수신)
```
M:PWM,V:DIR,A:SERVO\n
```
예:
```
M:600,V:1,A:1400\n
```

---

## 🛠️ API 참고

### 모터 (encoder.h)
```c
void Encoder_Init(TIM_HandleTypeDef *htim_pwm);
void Motor_SetSpeed(uint16_t pwm, uint8_t direction);  // direction: MOTOR_FORWARD(1), MOTOR_BACKWARD(0), MOTOR_STOP(2)
void Motor_Stop(void);
uint16_t Motor_GetPWM(void);
uint8_t Motor_GetDirection(void);
```

### 서보 (servo.h)
```c
void Servo_Init(TIM_HandleTypeDef *htim);
void Servo_SetAngle(int16_t angle_deg);    // -45 ~ +45
void Servo_SetPulse(uint16_t pulse_us);    // 1000 ~ 2000
void Servo_Center(void);
uint16_t Servo_GetPulse(void);
```

### 초음파 (supersonic.h)
```c
void Supersonic_Init(TIM_HandleTypeDef *htim);
void Supersonic_Trigger(void);              // 트리거 신호 발생
float Supersonic_GetDistance(void);         // cm 단위 거리 반환
```

### UART (serial_communication.h)
```c
void Serial_Init(UART_HandleTypeDef *huart);
void Serial_SendAll(void);                  // 모든 센서 데이터 전송
void Serial_SendMotor(void);
void Serial_SendServo(void);
void Serial_SendDistance(void);
void Serial_RxCallback(void);               // 수신 콜백 (자동 호출)
```

---

## 🔧 트러블슈팅

| 증상 | 원인 | 해결책 |
|------|------|--------|
| 모터가 안 움직임 | PWM 출력 미설정 | TIM2 설정 확인, 모터 드라이버 연결 확인 |
| 서보가 안 움직임 | 50Hz 신호 미발생 | TIM3 설정 확인, 펄스값 범위 확인 (1000~2000) |
| 초음파가 측정 안 함 | 핀 연결 오류 | PB6 (ECHO) 연결 확인, TIM4 IC 설정 확인 |
| UART 통신 안 됨 | 포트/보드레이트 오류 | 포트 번호(115200), 케이블 확인 |
| 테스트 안 나타남 | 테스트 플래그 미설정 | `test.h`에서 해당 플래그를 1로 변경 후 다시 빌드 |

---

## 💡 팁

1. **시리얼 모니터 추천:** 
   - PuTTY (Windows)
   - CoolTerm (Cross-platform)
   - STM32CubeIDE 내장 시리얼 뷰어

2. **각 테스트는 독립적:** 하나씩 테스트해서 문제 원인 파악 용이

3. **LED 피드백:** LD2는 초음파/ALL 모드에서 20cm 이내 감지 시 점등

4. **타임아웃 방지:** 테스트 후 100ms 이상 대기 추천

5. **정상 모드 루프:**
   ```c
   while(1) {
       Supersonic_Trigger();           // 60ms
       Serial_SendAll();               // 100ms
       if (distance < 20cm) Motor_Stop();
   }
   ```

---

## 📞 추가 정보

- **MCU 사양:** STM32F446RETx (180MHz, 256KB RAM, 512KB Flash)
- **HAL 버전:** STM32CubeF4
- **IDE:** STM32CubeIDE 1.12+
- **컴파일러:** GCC ARM

---

**작성일:** 2026-05-19  
**프로젝트:** Platoon Vehicle Control System  
**상태:** ✅ 테스트 완료
