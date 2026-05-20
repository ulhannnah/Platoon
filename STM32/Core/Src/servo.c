#include "servo.h"

static TIM_HandleTypeDef *_htim = NULL;
static uint16_t _current_pulse  = SERVO_CENTER;

void Servo_Init(TIM_HandleTypeDef *htim)
{
    _htim = htim;
    HAL_TIM_PWM_Start(_htim, TIM_CHANNEL_1);
    Servo_Center();
}

/* 각도(degree) → 펄스(us) 변환 후 출력 */
void Servo_SetAngle(int16_t angle_deg)
{
    // -45° ~ +45° 클램핑
    if (angle_deg < -45) angle_deg = -45;
    if (angle_deg >  45) angle_deg =  45;

    // 선형 매핑: -45 → 1000us, 0 → 1500us, +45 → 2000us
    uint16_t pulse = (uint16_t)(SERVO_CENTER + (angle_deg * 500 / 45));
    Servo_SetPulse(pulse);
}

/* 펄스(us) 직접 설정 */
void Servo_SetPulse(uint16_t pulse_us)
{
    if (pulse_us < SERVO_LEFT_MAX)  pulse_us = SERVO_LEFT_MAX;
    if (pulse_us > SERVO_RIGHT_MAX) pulse_us = SERVO_RIGHT_MAX;

    _current_pulse = pulse_us;
    __HAL_TIM_SET_COMPARE(_htim, TIM_CHANNEL_1, pulse_us);
}

void Servo_Center(void)
{
    Servo_SetPulse(SERVO_CENTER);
}

uint16_t Servo_GetPulse(void)
{
    return _current_pulse;
}
