#ifndef SUPERSONIC_H
#define SUPERSONIC_H

#include "main.h"

/* ── 설정 ── */
#define SUPERSONIC_TIMEOUT_US  38000  // 최대 측정 시간 (38ms)
#define SUPERSONIC_MIN_CM      2.0f   // 최소 측정 거리
#define SUPERSONIC_MAX_CM      400.0f // 최대 측정 거리

/* ── 함수 선언 ── */
void     Supersonic_Init(TIM_HandleTypeDef *htim);
void     Supersonic_Trigger(void);
void     Supersonic_CaptureCallback(TIM_HandleTypeDef *htim);
float    Supersonic_GetDistance(void);
uint8_t  Supersonic_IsReady(void);

#endif /* SUPERSONIC_H */
