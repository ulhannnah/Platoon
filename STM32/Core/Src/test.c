#include "test.h"
#include "servo.h"
#include "encoder.h"
#include "supersonic.h"
#include "serial_communication.h"
#include <stdio.h>
#include <string.h>

/* ─────────────────────────────────────────────────────────
   외부 변수 선언 (main.c에서 정의됨)
   ───────────────────────────────────────────────────────── */
extern UART_HandleTypeDef huart2;

/* ═══════════════════════════════════════════════════════════════════════════
   테스트 1: 모터 제어 테스트
   
   기능: 모터 속도와 방향을 체계적으로 테스트
   - 전진 (PWM 0 → 500 → 999)
   - 일시정지
   - 후진 (PWM 0 → 500 → 999)
   ═══════════════════════════════════════════════════════════════════════════ */
void Test_Motor(void)
{
    char buf[64];
    
    HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n=== MOTOR TEST ===\r\n", 23, 100);
    HAL_Delay(500);

    /* ─ 전진: PWM 점진적 증가 ─ */
    HAL_UART_Transmit(&huart2, (uint8_t*)"[FWD] Speed increasing...\r\n", 28, 100);
    for (uint16_t pwm = 0; pwm <= 999; pwm += 100) {
        Motor_SetSpeed(pwm, MOTOR_FORWARD);
        snprintf(buf, sizeof(buf), "PWM: %d (FWD)\r\n", pwm);
        HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 100);
        HAL_Delay(500);
    }

    /* ─ 정지 ─ */
    Motor_Stop();
    HAL_UART_Transmit(&huart2, (uint8_t*)"[STOP] Motor stopped.\r\n", 24, 100);
    HAL_Delay(1000);

    /* ─ 후진: PWM 점진적 증가 ─ */
    HAL_UART_Transmit(&huart2, (uint8_t*)"[REV] Speed increasing...\r\n", 28, 100);
    for (uint16_t pwm = 0; pwm <= 999; pwm += 100) {
        Motor_SetSpeed(pwm, MOTOR_BACKWARD);
        snprintf(buf, sizeof(buf), "PWM: %d (REV)\r\n", pwm);
        HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 100);
        HAL_Delay(500);
    }

    /* ─ 최종 정지 ─ */
    Motor_Stop();
    HAL_UART_Transmit(&huart2, (uint8_t*)"[END] Motor test complete.\r\n", 29, 100);
    HAL_Delay(1000);
}

/* ═══════════════════════════════════════════════════════════════════════════
   테스트 2: 서보 각도 테스트
   
   기능: 서보를 좌/우로 스윕
   - 중앙(0°) → 좌(-45°) → 우(+45°) → 중앙 반복
   ═══════════════════════════════════════════════════════════════════════════ */
void Test_Servo(void)
{
    char buf[64];
    
    HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n=== SERVO TEST ===\r\n", 23, 100);
    HAL_Delay(500);

    /* ─ 중앙으로 이동 ─ */
    Servo_Center();
    HAL_UART_Transmit(&huart2, (uint8_t*)"Center (0 deg)\r\n", 16, 100);
    HAL_Delay(1000);

    /* ─ 각도 스윕: -45° → +45° → -45° (3회) ─ */
    for (int cycle = 0; cycle < 3; cycle++) {
        /* 좌 이동 */
        for (int16_t angle = 0; angle >= -45; angle -= 15) {
            Servo_SetAngle(angle);
            snprintf(buf, sizeof(buf), "Angle: %d deg (pulse: %u us)\r\n", 
                     angle, Servo_GetPulse());
            HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 100);
            HAL_Delay(300);
        }
        HAL_Delay(500);

        /* 우 이동 */
        for (int16_t angle = -45; angle <= 45; angle += 15) {
            Servo_SetAngle(angle);
            snprintf(buf, sizeof(buf), "Angle: %d deg (pulse: %u us)\r\n", 
                     angle, Servo_GetPulse());
            HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 100);
            HAL_Delay(300);
        }
        HAL_Delay(500);

        /* 좌 이동 */
        for (int16_t angle = 45; angle >= -45; angle -= 15) {
            Servo_SetAngle(angle);
            snprintf(buf, sizeof(buf), "Angle: %d deg (pulse: %u us)\r\n", 
                     angle, Servo_GetPulse());
            HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 100);
            HAL_Delay(300);
        }
        HAL_Delay(500);
    }

    /* ─ 중앙으로 복귀 ─ */
    Servo_Center();
    HAL_UART_Transmit(&huart2, (uint8_t*)"[END] Servo test complete. Back to center.\r\n", 45, 100);
    HAL_Delay(1000);
}

/* ═══════════════════════════════════════════════════════════════════════════
   테스트 3: 초음파 센서 테스트
   
   기능: 초음파 거리 계속 측정 및 장애물 감지 테스트
   - 30초 동안 주기적으로 거리 측정
   - 20cm 미만 → 경고
   ═══════════════════════════════════════════════════════════════════════════ */
void Test_Ultrasonic(void)
{
    char buf[64];
    uint32_t start_time = HAL_GetTick();
    uint32_t test_duration = 30000;  // 30초
    
    HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n=== ULTRASONIC TEST ===\r\n", 28, 100);
    HAL_UART_Transmit(&huart2, (uint8_t*)"Measuring for 30 seconds...\r\n", 30, 100);
    HAL_Delay(500);

    while (HAL_GetTick() - start_time < test_duration) {
        Supersonic_Trigger();
        HAL_Delay(60);

        float distance = Supersonic_GetDistance();
        
        if (distance > 0.0f) {
            snprintf(buf, sizeof(buf), "Distance: %.1f cm", distance);
            
            if (distance < 20.0f) {
                strcat(buf, " [WARNING: OBSTACLE NEAR!]\r\n");
                HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);  // LED ON
            } else {
                strcat(buf, "\r\n");
                HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);  // LED OFF
            }
        } else {
            snprintf(buf, sizeof(buf), "Distance: ERROR (no echo)\r\n");
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
        }
        
        HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 100);
        HAL_Delay(500);
    }

    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);  // LED OFF
    HAL_UART_Transmit(&huart2, (uint8_t*)"[END] Ultrasonic test complete.\r\n", 34, 100);
    HAL_Delay(1000);
}

/* ═══════════════════════════════════════════════════════════════════════════
   테스트 4: UART 통신 테스트
   
   기능: 수신/송신 루프 테스트
   - 주기적으로 모든 센서 데이터 송신
   - 명령어 수신 후 적용 (M:PWM,V:DIR,A:PULSE)
   ═══════════════════════════════════════════════════════════════════════════ */
void Test_Serial(void)
{
    char buf[64];
    uint32_t last_send = 0;
    uint32_t test_duration = 20000;  // 20초
    uint32_t start_time = HAL_GetTick();
    
    HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n=== SERIAL TEST ===\r\n", 24, 100);
    HAL_UART_Transmit(&huart2, (uint8_t*)"Transmitting sensor data every 500ms...\r\n", 42, 100);
    HAL_UART_Transmit(&huart2, (uint8_t*)"Try sending: M:500,V:1,A:1500\r\n", 33, 100);
    HAL_Delay(500);

    while (HAL_GetTick() - start_time < test_duration) {
        uint32_t now = HAL_GetTick();

        // 주기적 전송 (500ms)
        if (now - last_send >= 500) {
            snprintf(buf, sizeof(buf),
                     "[TX] PWM:%d,DIR:%d,SERVO:%d,DIST:%.1f\r\n",
                     Motor_GetPWM(),
                     Motor_GetDirection(),
                     Servo_GetPulse(),
                     Supersonic_GetDistance());
            HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 100);
            last_send = now;
        }

        // 초음파 트리거 (60ms)
        Supersonic_Trigger();
        HAL_Delay(60);
    }

    HAL_UART_Transmit(&huart2, (uint8_t*)"[END] Serial test complete.\r\n", 30, 100);
    HAL_Delay(1000);
}

/* ═══════════════════════════════════════════════════════════════════════════
   테스트 5: 모든 센서 한번에 체크
   
   기능: 모터, 서보, 초음파, UART 동시에 작동 확인
   - 간단한 주기로 모든 기능 병렬 실행
   ═══════════════════════════════════════════════════════════════════════════ */
void Test_All(void)
{
    char buf[64];
    uint32_t start_time = HAL_GetTick();
    uint32_t test_duration = 20000;  // 20초
    uint32_t last_motor_change = 0;
    uint32_t last_servo_change = 0;
    uint32_t last_ultrasonic = 0;
    uint32_t last_send = 0;
    int16_t servo_angle = 0;
    int8_t servo_direction = 1;  // 1: 증가, -1: 감소
    uint8_t motor_pwm_step = 0;

    HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n=== COMPREHENSIVE TEST ===\r\n", 32, 100);
    HAL_UART_Transmit(&huart2, (uint8_t*)"All sensors running simultaneously...\r\n", 39, 100);
    HAL_Delay(500);

    while (HAL_GetTick() - start_time < test_duration) {
        uint32_t now = HAL_GetTick();

        /* ─ 모터 제어 (2초마다 PWM 변경) ─ */
        if (now - last_motor_change >= 2000) {
            uint16_t pwm = motor_pwm_step * 250;
            if (motor_pwm_step % 2 == 0) {
                Motor_SetSpeed(pwm, MOTOR_FORWARD);
            } else {
                Motor_SetSpeed(pwm, MOTOR_BACKWARD);
            }
            motor_pwm_step = (motor_pwm_step + 1) % 4;
            last_motor_change = now;
        }

        /* ─ 서보 제어 (1초마다 각도 변경) ─ */
        if (now - last_servo_change >= 1000) {
            servo_angle += (15 * servo_direction);
            if (servo_angle >= 45) {
                servo_angle = 45;
                servo_direction = -1;
            } else if (servo_angle <= -45) {
                servo_angle = -45;
                servo_direction = 1;
            }
            Servo_SetAngle(servo_angle);
            last_servo_change = now;
        }

        /* ─ 초음파 트리거 (60ms) ─ */
        if (now - last_ultrasonic >= 60) {
            Supersonic_Trigger();
            last_ultrasonic = now;
        }

        /* ─ 데이터 송신 (1초마다) ─ */
        if (now - last_send >= 1000) {
            float dist = Supersonic_GetDistance();
            snprintf(buf, sizeof(buf),
                     "[ALL] M:%d,D:%d,S:%d,DIST:%.1f\r\n",
                     Motor_GetPWM(),
                     Motor_GetDirection(),
                     Servo_GetPulse(),
                     dist);
            HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 100);
            last_send = now;

            // 장애물 감지 피드백
            if (dist > 0.0f && dist < 20.0f) {
                HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
            } else {
                HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
            }
        }

        HAL_Delay(10);  // CPU 부하 감소
    }

    /* ─ 정리 ─ */
    Motor_Stop();
    Servo_Center();
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    HAL_UART_Transmit(&huart2, (uint8_t*)"[END] Comprehensive test complete.\r\n", 37, 100);
    HAL_Delay(1000);
}
