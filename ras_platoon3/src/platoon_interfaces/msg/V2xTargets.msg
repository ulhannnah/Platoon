# V2xTargets.msg
# ESP32-S3 -> Pi. "ESP32-S3 V2X 인터페이스 정리.md" §7.2 v2x_targets 패킷 전체.
# v2x_node.py가 USB CDC JSON Line을 파싱해서 이 메시지로 발행한다 (/v2x/targets).

int32 seq
int64 timestamp_ms
int32 self_vehicle_id

V2xTarget[] targets
