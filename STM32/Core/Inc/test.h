#ifndef TEST_H
#define TEST_H

#include "main.h"

/* ─────────────────────────────────────────────────────────
   테스트 모드 선택 (아래 중 하나만 활성화)
   ───────────────────────────────────────────────────────── */
#define TEST_MODE_MOTOR        0   // 모터 테스트
#define TEST_MODE_SERVO        0   // 서보 테스트  
#define TEST_MODE_ULTRASONIC   0   // 초음파 센서 테스트
#define TEST_MODE_SERIAL       0   // UART 통신 테스트
#define TEST_MODE_ALL          0   // 모든 센서 한번에 체크

/* ─────────────────────────────────────────────────────────
   테스트 함수 선언
   ───────────────────────────────────────────────────────── */
void Test_Motor(void);
void Test_Servo(void);
void Test_Ultrasonic(void);
void Test_Serial(void);
void Test_All(void);

#endif /* TEST_H */
