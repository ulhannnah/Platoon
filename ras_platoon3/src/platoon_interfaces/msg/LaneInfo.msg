# LaneInfo.msg

# LaneStyle (LaneInfo.msg 내부에 포함 또는 별도 정의)
uint8 UNKNOWN = 0
uint8 SOLID = 1
uint8 DASHED = 2

int32 offset
bool lane_detected

# 차선 점선/실선 스타일 (0: UNKNOWN, 1: SOLID, 2: DASHED)
int8 left_style
int8 right_style

# 전향 보상(Feed-Forward) 연산을 위한 차선 기울기
float32 left_slope
float32 right_slope

# 차선 폭
float32 dynamic_virtual_offset