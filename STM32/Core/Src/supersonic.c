#include "supersonic.h"

static TIM_HandleTypeDef *_htim      = NULL;
static volatile uint32_t  _echo_start = 0;
static volatile uint32_t  _echo_end   = 0;
static volatile uint8_t   _ready_flag = 0;  // 1 = 측정 완료
static volatile float     _distance   = 0.0f;

void Supersonic_Init(TIM_HandleTypeDef *htim)
{
    _htim = htim;
    HAL_TIM_IC_Start_IT(_htim, TIM_CHANNEL_1);
}

/* TRIG 신호 발사 - 메인 루프에서 60ms 간격으로 호출 */
void Supersonic_Trigger(void)
{
    _ready_flag = 0;
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
    HAL_Delay(1);   // 최소 10us (HAL_Delay 최소 1ms)
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
}

/* TIM4 Input Capture 콜백에서 호출 */
void Supersonic_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != _htim->Instance) return;

    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET) {
        // 상승 엣지 → 시작
        _echo_start = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    } else {
        // 하강 엣지 → 종료
        _echo_end = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

        // 오버플로우 처리
        uint32_t duration;
        if (_echo_end >= _echo_start) {
            duration = _echo_end - _echo_start;
        } else {
            duration = (65535 - _echo_start) + _echo_end;
        }

        // 거리 계산: us / 58 = cm (음속 340m/s 기준)
        float dist = duration / 58.0f;

        if (dist >= SUPERSONIC_MIN_CM && dist <= SUPERSONIC_MAX_CM) {
            _distance   = dist;
            _ready_flag = 1;
        }
    }
}

float Supersonic_GetDistance(void)
{
    return _distance;
}

uint8_t Supersonic_IsReady(void)
{
    if (_ready_flag) {
        _ready_flag = 0;
        return 1;
    }
    return 0;
}
