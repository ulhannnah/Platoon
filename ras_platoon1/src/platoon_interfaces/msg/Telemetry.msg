# STM32 Telemetry Message
int8    steer_deg       # 서보모터 현재 조향 각도 (-70 ~ +70 deg)
int16   left_delta      # 좌측 바퀴 엔코더 델타 (signed 16-bit)
int16   right_delta     # 우측 바퀴 엔코더 델타 (signed 16-bit)
uint16  dist_cm         # 초음파 거리 측정값 (cm, unsigned 16-bit)
float32 speed_mps       # left_delta/right_delta를 엔코더 CPR·바퀴 지름·경과시간으로
                         # 환산한 실제 속도 (m/s). control_node.py가 계산해서 채움 —
                         # PID 제어에 안 쓰더라도(지금은 고정 듀티 방식) 다른 노드가
                         # "지금 실제로 얼마나 빠른가"를 알아야 할 때(V2X self_status 등)
                         # 이 필드를 쓴다.