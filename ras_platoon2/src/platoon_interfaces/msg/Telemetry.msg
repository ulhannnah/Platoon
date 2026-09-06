# STM32 Telemetry Message
int8    steer_deg       # 서보모터 현재 조향 각도 (-70 ~ +70 deg)
int16   left_delta      # 좌측 바퀴 엔코더 델타 (signed 16-bit)
int16   right_delta     # 우측 바퀴 엔코더 델타 (signed 16-bit)
uint16  dist_cm         # 초음파 거리 측정값 (cm, unsigned 16-bit)