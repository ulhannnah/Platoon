#ifndef SERIAL_COMMUNICATION_H
#define SERIAL_COMMUNICATION_H

#include "main.h"
#include "encoder.h"
#include "servo.h"
#include "supersonic.h"

/* ── 패킷 형식 (STM32 → Jetson) ──────────────────────────
   "PWM:500,DIR:1,SERVO:1500,DIST:35.2\n"
   PWM   : 모터 PWM (0~999)
   DIR   : 방향 (1=전진, 0=후진)
   SERVO : 서보 펄스 (1000~2000)
   DIST  : 초음파 거리 (cm)
────────────────────────────────────────────────────────── */

/* ── 함수 선언 ── */
void Serial_Init(UART_HandleTypeDef *huart);
void Serial_SendMotor(void);        // 모터 PWM + 방향 전송
void Serial_SendServo(void);        // 서보 위치 전송
void Serial_SendDistance(void);     // 초음파 거리 전송
void Serial_SendAll(void);          // 전체 데이터 한번에 전송
void Serial_RxCallback(void);       // UART 수신 콜백에서 호출

#endif /* SERIAL_COMMUNICATION_H */
