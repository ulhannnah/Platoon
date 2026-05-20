#ifndef SERVO_H
#define SERVO_H

#include "main.h"

/* ── 서보 각도 범위 (us 단위) ── */
#define SERVO_CENTER   1500   // 중앙 (0°)
#define SERVO_LEFT_MAX 1000   // 최대 좌회전 (-45°)
#define SERVO_RIGHT_MAX 2000  // 최대 우회전 (+45°)

/* ── 함수 선언 ── */
void Servo_Init(TIM_HandleTypeDef *htim);
void Servo_SetAngle(int16_t angle_deg);   // -45 ~ +45
void Servo_SetPulse(uint16_t pulse_us);   // 1000 ~ 2000
void Servo_Center(void);
uint16_t Servo_GetPulse(void);

#endif /* SERVO_H */
