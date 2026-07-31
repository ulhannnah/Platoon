# RPi5 ↔ STM32 UART 프로토콜 명세

플래툰(군집주행) 프로젝트에서 라즈베리파이5와 STM32 사이에 주고받는 패킷 정의와
STM32 쪽 송수신 구현 코드입니다. 이 문서대로 구현하면 RPi 쪽(`stm32_interface.py`)과
바로 맞물립니다.

---

## 1. 역할 경계

| | 담당 |
|---|---|
| **RPi5** | 판단·계획. **목표 속도(m/s)**와 **목표 조향각(rad)** 까지만 계산해서 내려보냄 |
| **STM32** | 실행. 목표값을 받아 **엔코더 피드백 + PID**로 모터 PWM 제어, 서보 각도 변환 |

PID 게인, PWM 듀티 계산, 아커만 조향 기하 보정은 **전부 STM32 담당**입니다.
RPi는 PWM이라는 단어조차 모릅니다.

반대로 STM32는 플래툰 상태(리더/팔로워, 차간거리 목표 등)를 알 필요가 없습니다.
`mode` 필드를 받긴 하지만 이건 **안전 로직 분기용**이지 제어 판단용이 아닙니다.

---

## 2. 물리 계층 설정

| 항목 | 값 |
|---|---|
| 인터페이스 | UART (USART) |
| Baudrate | **115200** |
| 데이터 | 8bit, No parity, 1 stop bit (8N1) |
| 흐름제어 | 없음 |
| 송신 주기 (RPi→STM32) | 약 50Hz (20ms) |
| 송신 주기 (STM32→RPi) | 약 50Hz (20ms) |

> Baudrate는 115200으로 시작하되, 패킷 유실이 잦으면 460800까지 올려도 됩니다.
> 양쪽 다 같은 값으로 맞춰야 합니다.

---

## 3. 프레임 구조

UART는 바이트가 유실되거나 중간부터 읽히면 구조체가 통째로 밀립니다.
**엉뚱한 값이 조향각으로 들어가면 사고**이므로 반드시 아래 프레이밍을 씁니다.

```text
┌──────┬──────┬─────┬──────────────────┬──────────┐
│ 0xAA │ 0x55 │ LEN │ PAYLOAD (LEN개)  │ CHECKSUM │
└──────┴──────┴─────┴──────────────────┴──────────┘
  헤더1  헤더2  길이      실제 데이터        검사합
```

- `LEN` : PAYLOAD의 바이트 수 (CHECKSUM 제외)
- `CHECKSUM` : **LEN 바이트 + PAYLOAD 전체**를 XOR한 값 (헤더는 제외)

수신 측은 헤더 2바이트(`0xAA 0x55`)를 찾을 때까지 바이트를 버리고,
체크섬이 틀리면 **그 패킷을 통째로 폐기**합니다. 절대 부분적으로 반영하지 않습니다.

---

## 4. 데이터 표현 — 고정소수점

`float`를 그대로 보내지 않고 **int16 고정소수점**을 씁니다.

| 이유 | 설명 |
|---|---|
| 크기 | 4byte → 2byte, 패킷 절반 |
| 안전 | `float`는 NaN이나 `1e30` 같은 쓰레기값이 그대로 넘어가 모터가 튐. int16은 범위가 강제로 제한됨 |
| 이식성 | 부동소수점 표현 차이 걱정 없음 |

| 물리량 | 전송 단위 | 변환 | 예시 |
|---|---|---|---|
| 속도 | mm/s (`int16`) | `m/s × 1000` | 0.5 m/s → `500` |
| 조향각 | 밀리라디안 (`int16`) | `rad × 1000` | 0.12 rad → `120` |
| 거리 | mm (`uint16`) | `m × 1000` | 0.8 m → `800` |

`int16` 범위는 -32768~32767이므로 속도는 ±32.7 m/s, 조향각은 ±32.7 rad까지
표현 가능합니다. 실제 차량 범위보다 훨씬 넓으니 충분합니다.

---

## 5. 패킷 정의

### 5.1 공통 상수

```c
#ifndef PLATOON_PROTOCOL_H
#define PLATOON_PROTOCOL_H

#include <stdint.h>

#define PKT_HEADER_1        0xAA
#define PKT_HEADER_2        0x55
#define PKT_MAX_PAYLOAD     32

/* 메시지 타입 */
#define MSG_DRIVE_COMMAND   0x01    /* RPi  -> STM32 */
#define MSG_DRIVE_FEEDBACK  0x81    /* STM32 -> RPi  */

/* 주행 모드 (안전 로직 분기용) */
#define MODE_SOLO_DRIVE       0
#define MODE_PLATOON_JOIN     1
#define MODE_PLATOON_MAINTAIN 2
#define MODE_PLATOON_EXIT     3
#define MODE_EMERGENCY_STOP   0xFF

/* STM32 상태 비트플래그 */
#define STATUS_MOTOR_ENABLED  (1 << 0)
#define STATUS_FAILSAFE       (1 << 1)   /* RPi 무수신으로 정지 중 */
#define STATUS_OBSTACLE       (1 << 2)   /* 초음파 근접 감지 */

/* 실패 안전: RPi로부터 이 시간 이상 패킷이 없으면 정지 */
#define FAILSAFE_TIMEOUT_MS   200

#endif /* PLATOON_PROTOCOL_H */
```

### 5.2 RPi → STM32 : 주행 명령

```c
/* 7 bytes */
typedef struct __attribute__((packed)) {
    uint8_t  msg_type;              /* MSG_DRIVE_COMMAND (0x01) */
    uint8_t  mode;                  /* MODE_* */
    int16_t  target_speed_mmps;     /* 목표 속도 (mm/s) */
    int16_t  target_steer_mrad;     /* 목표 조향각 (밀리라디안), + = 좌회전 */
    uint8_t  seq;                   /* 순서번호 0~255 순환, 유실 감지용 */
} DriveCommand;
```

**조향각 부호 규약**: 반시계 방향(좌회전)이 양수입니다. 이게 실제 서보 방향과
반대면 STM32 쪽에서 부호를 뒤집어 주세요. RPi 코드는 수정하지 않습니다.

**`mode` 활용**: `MODE_EMERGENCY_STOP`을 받으면 목표속도와 무관하게 즉시 정지하세요.
`MODE_PLATOON_MAINTAIN`처럼 차간거리가 좁은 모드에서는 초음파 임계값을
더 보수적으로 잡아도 좋습니다.

### 5.3 STM32 → RPi : 상태 피드백

```c
/* 9 bytes */
typedef struct __attribute__((packed)) {
    uint8_t  msg_type;              /* MSG_DRIVE_FEEDBACK (0x81) */
    uint8_t  status;                /* STATUS_* 비트플래그 */
    int16_t  current_speed_mmps;    /* 엔코더 기반 실제 속도 (mm/s) */
    int16_t  current_steer_mrad;    /* 현재 조향각 (밀리라디안) */
    uint16_t front_distance_mm;     /* 초음파 전방거리 (mm), 미측정 시 0xFFFF */
    uint8_t  seq;                   /* 마지막으로 정상 수신한 명령의 seq */
} DriveFeedback;
```

**이 패킷이 왜 꼭 필요한가**

RPi의 플래툰 알고리즘이 이 값들 없이는 계산이 안 됩니다.

- `current_speed_mmps` → 적합도 계산의 속도 유사도 점수, 그리고 팔로워 목표속도식
  `v_target = v_L + K_d(d - d_target) + K_v(v_L - v_F)` 의 `v_F`
- `front_distance_mm` → 설계 문서상 `PLATOON_MAINTAIN` 단계의 **주 거리 센서는 초음파**
  (UWB는 보조). 초음파가 STM32에 연결돼 있다면 이 필드로 올려주세요.
- `seq` → RPi가 자기 명령이 실제로 도달했는지 확인

> 초음파가 STM32가 아니라 RPi에 직접 연결돼 있다면 `front_distance_mm`은
> `0xFFFF`로 두고 알려주세요. RPi 쪽에서 직접 읽도록 바꾸면 됩니다.

---

## 6. STM32 구현 코드

### 6.1 체크섬

```c
static uint8_t calc_checksum(uint8_t len, const uint8_t *payload)
{
    uint8_t cs = len;
    for (uint8_t i = 0; i < len; i++) {
        cs ^= payload[i];
    }
    return cs;
}
```

### 6.2 수신 — 상태머신 파서

바이트가 한 개씩 들어와도 동작하는 파서입니다.
UART 인터럽트(또는 DMA 링버퍼)에서 받은 바이트를 하나씩 `parser_feed()`에 넣으세요.

```c
#include <string.h>
#include "platoon_protocol.h"

typedef enum {
    WAIT_HEADER_1 = 0,
    WAIT_HEADER_2,
    WAIT_LEN,
    WAIT_PAYLOAD,
    WAIT_CHECKSUM
} ParserState;

typedef struct {
    ParserState state;
    uint8_t     len;
    uint8_t     idx;
    uint8_t     payload[PKT_MAX_PAYLOAD];
} PacketParser;

static PacketParser g_parser;

/* 새로 완성된 명령을 담을 곳 */
volatile DriveCommand g_last_command;
volatile uint32_t     g_last_command_tick = 0;
volatile uint8_t      g_command_ready = 0;

static void on_packet_received(uint8_t len, const uint8_t *payload)
{
    if (len < 1) return;

    if (payload[0] == MSG_DRIVE_COMMAND && len == sizeof(DriveCommand)) {
        DriveCommand cmd;
        memcpy(&cmd, payload, sizeof(DriveCommand));

        g_last_command      = cmd;
        g_last_command_tick = HAL_GetTick();
        g_command_ready     = 1;
    }
    /* 다른 msg_type은 무시 */
}

void parser_feed(uint8_t byte)
{
    switch (g_parser.state) {

    case WAIT_HEADER_1:
        if (byte == PKT_HEADER_1) {
            g_parser.state = WAIT_HEADER_2;
        }
        break;

    case WAIT_HEADER_2:
        if (byte == PKT_HEADER_2) {
            g_parser.state = WAIT_LEN;
        } else if (byte == PKT_HEADER_1) {
            /* 0xAA 0xAA 0x55 같은 경우 대비 — 상태 유지 */
        } else {
            g_parser.state = WAIT_HEADER_1;
        }
        break;

    case WAIT_LEN:
        if (byte == 0 || byte > PKT_MAX_PAYLOAD) {
            g_parser.state = WAIT_HEADER_1;   /* 비정상 길이, 폐기 */
        } else {
            g_parser.len   = byte;
            g_parser.idx   = 0;
            g_parser.state = WAIT_PAYLOAD;
        }
        break;

    case WAIT_PAYLOAD:
        g_parser.payload[g_parser.idx++] = byte;
        if (g_parser.idx >= g_parser.len) {
            g_parser.state = WAIT_CHECKSUM;
        }
        break;

    case WAIT_CHECKSUM:
        if (byte == calc_checksum(g_parser.len, g_parser.payload)) {
            on_packet_received(g_parser.len, g_parser.payload);
        }
        /* 체크섬 불일치면 통째로 폐기 — 부분 반영 절대 금지 */
        g_parser.state = WAIT_HEADER_1;
        break;

    default:
        g_parser.state = WAIT_HEADER_1;
        break;
    }
}
```

UART 인터럽트 콜백 예시 (HAL 기준):

```c
static uint8_t g_rx_byte;

void uart_start_receive(void)
{
    HAL_UART_Receive_IT(&huart2, &g_rx_byte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        parser_feed(g_rx_byte);
        HAL_UART_Receive_IT(&huart2, &g_rx_byte, 1);  /* 다음 바이트 대기 */
    }
}
```

### 6.3 송신

```c
static uint8_t g_tx_seq = 0;

static void send_packet(uint8_t len, const uint8_t *payload)
{
    uint8_t frame[3 + PKT_MAX_PAYLOAD + 1];
    uint8_t n = 0;

    frame[n++] = PKT_HEADER_1;
    frame[n++] = PKT_HEADER_2;
    frame[n++] = len;
    memcpy(&frame[n], payload, len);
    n += len;
    frame[n++] = calc_checksum(len, payload);

    HAL_UART_Transmit(&huart2, frame, n, 50);
}

void send_feedback(int16_t speed_mmps, int16_t steer_mrad,
                   uint16_t front_mm, uint8_t status, uint8_t ack_seq)
{
    DriveFeedback fb;
    fb.msg_type            = MSG_DRIVE_FEEDBACK;
    fb.status              = status;
    fb.current_speed_mmps  = speed_mmps;
    fb.current_steer_mrad  = steer_mrad;
    fb.front_distance_mm   = front_mm;
    fb.seq                 = ack_seq;

    send_packet(sizeof(fb), (const uint8_t *)&fb);
}
```

> `HAL_UART_Transmit`은 블로킹이라 제어 루프를 막을 수 있습니다.
> 제어 주기가 빡빡해지면 `HAL_UART_Transmit_DMA`로 바꾸세요.

### 6.4 실패 안전 (필수)

**이게 없으면 RPi가 죽거나 USB가 빠졌을 때 STM32는 마지막 명령("0.5m/s 전진")을
영원히 유지합니다. 차가 그대로 벽으로 갑니다.** 군집주행은 차간거리가 좁아서
특히 위험합니다.

```c
void control_loop(void)   /* 1kHz 타이머 인터럽트 등에서 호출 */
{
    int16_t target_speed;
    int16_t target_steer;
    uint8_t status = 0;

    uint32_t now = HAL_GetTick();

    if (now - g_last_command_tick > FAILSAFE_TIMEOUT_MS) {
        /* RPi 무수신 → 정지 */
        target_speed = 0;
        target_steer = 0;
        status |= STATUS_FAILSAFE;
    } else if (g_last_command.mode == MODE_EMERGENCY_STOP) {
        target_speed = 0;
        target_steer = g_last_command.target_steer_mrad;
    } else {
        target_speed = g_last_command.target_speed_mmps;
        target_steer = g_last_command.target_steer_mrad;
        status |= STATUS_MOTOR_ENABLED;
    }

    /* 여기부터가 STM32 고유 영역 */
    motor_pid_update(target_speed);      /* 엔코더 피드백 + PID → PWM */
    servo_set_angle(target_steer);       /* 밀리라디안 → 서보 각도 변환 */
}
```

---

## 7. 검증 방법

구현 후 아래 순서로 확인하면 문제를 빨리 잡을 수 있습니다.

1. **루프백 테스트** — STM32 TX와 RX를 점퍼로 연결하고, 자기가 보낸 피드백 패킷을
   자기 파서가 정상 파싱하는지 확인
2. **체크섬 오류 주입** — RPi에서 일부러 체크섬을 틀린 패킷을 보내고,
   STM32가 조용히 폐기하는지 (모터가 반응 안 하는지) 확인
3. **중간 절단 테스트** — 패킷 전송 중 케이블을 뽑았다 꽂아서,
   파서가 헤더를 다시 찾아 정상 복구되는지 확인
4. **실패 안전 테스트** — RPi에서 명령 송신을 멈추고 200ms 뒤 모터가
   실제로 멈추는지 확인 **(가장 중요, 반드시 확인)**

---

## 8. 확정 필요 항목

| 항목 | 현재 | 확정해야 할 것 |
|---|---|---|
| UART 포트 | `USART2` 가정 | STM32 실제 배선 확인 |
| 조향 부호 | 좌회전 = 양수 | 실제 서보 방향과 일치하는지 |
| 초음파 위치 | STM32 연결 가정 | RPi 직결이면 `front_distance_mm` 제거 |
| Baudrate | 115200 | 유실 잦으면 상향 |
| 축간거리 | RPi 쪽 `WHEELBASE_M` 임시값 | 실측값 필요 (pure pursuit 계산에 직접 영향) |