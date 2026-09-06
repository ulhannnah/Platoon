int32 seq
int64 timestamp_ms

int32 vehicle_id
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

# 플래툰 전체 즉시 전파용 — 초음파 기반 거리제어가 반응하길 기다리지 않고,
# 누군가(특히 리더) 비상정지하면 방송 즉시 전체가 같이 서게 하기 위함.
int32 emergency

# 이 차량이 decision_node에서 "지금 실제로" 내고 있는 speed_mode(0/1/2).
# speed_mps(엔코더 환산)와 달리 항상 신뢰 가능해서, 뒤차의 CACC 피드포워드
# 기준값으로 씀 (fsm_decision_node가 자기 vehicle_cmd를 구독해서 채움).
int32 speed_level
