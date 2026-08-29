#lane_detector 노드가 발행

# Lane Style
uint8 UNKNOWN=0
uint8 SOLID=1
uint8 DASHED=2

# Lane Information
int32 offset

bool lane_detected

float32 left_slope
float32 right_slope

uint8 left_style
uint8 right_style