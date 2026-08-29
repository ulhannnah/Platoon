# V2xTarget.msg
# ESP32-S3가 보내는 v2x_targets 패킷의 targets[] 배열 원소 하나 (문서 §7.2 그대로).
#
# leader_vehicle_id는 ESP32 원본 패킷엔 없는 필드다. v2x_node.py가 같은 패킷 안
# 다른 타겟 중 platoon_role==LEADER(1)인 걸 찾아서 채워 넣는 Pi 쪽 보강값이다 —
# platoon_fsm.py의 3대 이상 체인 로직(_setup_platoon)이 NearbyVehicle.leader_id를
# 요구하는데 원본 문서 스펙엔 이 필드가 없어서다. 같은 패킷 안에 리더가 안 잡히면
# -1(모름)로 남는다 — 이 경우 3대째 결합 판단이 부정확해질 수 있으니, ESP32 쪽에
# 타겟별 leader_vehicle_id 필드 추가를 요청하는 게 근본 해결책이다.

int32 vehicle_id
int32 uwb_id

float32 distance_m
float32 angle_deg
float32 rel_x_m
float32 rel_y_m

float32 speed_mps
float32 heading_deg

int32 driving_state
int32 platoon_state
int32 platoon_id
int32 platoon_enable
int32 platoon_role
int32 platoon_index

int32 uwb_valid
int32 espnow_valid
float32 confidence

int32 leader_vehicle_id
# Pi 쪽 보강값. -1이면 이번 패킷만으로는 알 수 없음 (위 설명 참고).
