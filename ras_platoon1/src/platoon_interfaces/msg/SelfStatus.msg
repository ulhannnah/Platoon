# SelfStatus.msg
# Pi -> ESP32-S3. "ESP32-S3 V2X 인터페이스 정리.md" §5 self_status 패킷 전체.
#
# fsm_decision_node.py(또는 decision_node.py)가 자기 판단 결과를 이 메시지로
# 발행하면(/v2x/self_status), v2x_node.py가 그대로 JSON으로 직렬화해서
# ESP32-S3로 주기적으로 보낸다. v2x_node.py는 이 값들의 의미를 몰라도 되고
# 그냥 전달만 한다 — FSM 지식은 fsm_decision_node.py 쪽에만 있으면 된다.

int32 seq
int64 timestamp_ms

int32 vehicle_id
int32 uwb_id
int32 destination_id

int32 driving_state
int32 platoon_state

float32 speed_mps
float32 heading_deg

int32 platoon_enable
int32 platoon_id
int32 platoon_role
int32 platoon_index

int32 leader_vehicle_id
int32 front_vehicle_id

float32 target_speed_mps
float32 target_gap_m
