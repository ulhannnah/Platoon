# V2xTarget.msg
# ESP32-S3가 보내는 v2x_targets 패킷의 targets[] 배열 원소 하나.
#
# leader_vehicle_id/front_vehicle_id는 이제 ESP32가 직접 계산해서 보내준다
# ("ESP32-S3 ↔ Raspberry Pi 통신 데이터 정리.md" §10.2 최신 정책). 예전엔
# v2x_node.py가 같은 패킷 안에서 리더를 추론하는 임시 보강 로직을 썼는데,
# 이제 그 필드를 그대로 받으면 된다 (혹시 구버전 펌웨어라 필드가 없으면
# v2x_node.py가 예전 추론 로직으로 폴백한다).
# uwb_id는 최신 정책상 Pi로 안 보내므로 이 메시지에 없다.

int32 vehicle_id

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
int32 front_vehicle_id
# 둘 다 -1이면 이번 패킷만으로는 알 수 없음 (구버전 펌웨어 폴백 실패 등).
