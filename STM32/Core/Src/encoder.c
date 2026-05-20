#include "encoder.h"

static TIM_HandleTypeDef *_htim_pwm = NULL;
static uint16_t _current_pwm        = 0;
static uint8_t  _current_dir        = MOTOR_STOP;

void Encoder_Init(TIM_HandleTypeDef *htim_pwm)
{
    _htim_pwm = htim_pwm;
    HAL_TIM_PWM_Start(_htim_pwm, TIM_CHANNEL_1);
    Motor_Stop();
}

/* 속도 + 방향 설정 */
void Motor_SetSpeed(uint16_t pwm, uint8_t direction)
{
    if (pwm > MOTOR_PWM_MAX) pwm = MOTOR_PWM_MAX;

    _current_pwm = pwm;
    _current_dir = direction;

    if (direction == MOTOR_FORWARD) {
        HAL_GPIO_WritePin(GPIOB, IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, IN2_Pin, GPIO_PIN_RESET);
    } else if (direction == MOTOR_BACKWARD) {
        HAL_GPIO_WritePin(GPIOB, IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, IN2_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOB, IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, IN2_Pin, GPIO_PIN_RESET);
        pwm = 0;
    }

    __HAL_TIM_SET_COMPARE(_htim_pwm, TIM_CHANNEL_1, pwm);
}

void Motor_Stop(void)
{
    Motor_SetSpeed(0, MOTOR_STOP);
}

uint16_t Motor_GetPWM(void)
{
    return _current_pwm;
}

uint8_t Motor_GetDirection(void)
{
    return _current_dir;
}
