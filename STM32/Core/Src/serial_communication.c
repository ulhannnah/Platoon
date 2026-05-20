#include "serial_communication.h"
#include <stdio.h>
#include <string.h>

static UART_HandleTypeDef *_huart  = NULL;
static uint8_t  _rx_byte           = 0;
static uint8_t  _rx_buf[64]        = {0};
static uint8_t  _rx_idx            = 0;

void Serial_Init(UART_HandleTypeDef *huart)
{
    _huart = huart;
    // 수신 인터럽트 시작
    HAL_UART_Receive_IT(_huart, &_rx_byte, 1);
}

/* ── 모터 PWM + 방향만 전송 ── */
void Serial_SendMotor(void)
{
    char buf[32];
    snprintf(buf, sizeof(buf),
             "PWM:%d,DIR:%d\n",
             Motor_GetPWM(),
             Motor_GetDirection());
    HAL_UART_Transmit(_huart, (uint8_t*)buf, strlen(buf), 100);
}

/* ── 서보 위치만 전송 ── */
void Serial_SendServo(void)
{
    char buf[24];
    snprintf(buf, sizeof(buf),
             "SERVO:%d\n",
             Servo_GetPulse());
    HAL_UART_Transmit(_huart, (uint8_t*)buf, strlen(buf), 100);
}

/* ── 초음파 거리만 전송 ── */
void Serial_SendDistance(void)
{
    char buf[24];
    snprintf(buf, sizeof(buf),
             "DIST:%.1f\n",
             Supersonic_GetDistance());
    HAL_UART_Transmit(_huart, (uint8_t*)buf, strlen(buf), 100);
}

/* ── 전체 데이터 한번에 전송 (메인 루프에서 주기적으로 호출) ── */
void Serial_SendAll(void)
{
    char buf[64];
    snprintf(buf, sizeof(buf),
             "PWM:%d,DIR:%d,SERVO:%d,DIST:%.1f\n",
             Motor_GetPWM(),
             Motor_GetDirection(),
             Servo_GetPulse(),
             Supersonic_GetDistance());
    HAL_UART_Transmit(_huart, (uint8_t*)buf, strlen(buf), 100);
}

/* ── UART 수신 콜백에서 호출 ──────────────────────────────
   Jetson → STM32 명령 형식:
   "M:500,V:1,A:1500\n"
    M = 모터 PWM, V = 방향, A = 서보 펄스
────────────────────────────────────────────────────────── */
void Serial_RxCallback(void)
{
    if (_rx_byte == '\n') {
        // 패킷 파싱
        int m = 0, v = 0, a = 0;
        if (sscanf((char*)_rx_buf, "M:%d,V:%d,A:%d", &m, &v, &a) == 3) {
            Motor_SetSpeed((uint16_t)m, (uint8_t)v);
            Servo_SetPulse((uint16_t)a);
        }
        // 버퍼 초기화
        memset(_rx_buf, 0, sizeof(_rx_buf));
        _rx_idx = 0;
    } else {
        if (_rx_idx < sizeof(_rx_buf) - 1)
            _rx_buf[_rx_idx++] = _rx_byte;
    }
    // 다음 수신 대기
    HAL_UART_Receive_IT(_huart, &_rx_byte, 1);
}
