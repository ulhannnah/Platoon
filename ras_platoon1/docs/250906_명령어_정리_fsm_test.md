# 실행 명령어 정리 (fsm_test.py 기반 — 촬영 시나리오용)

`250902_실행_명령어_정리.md`는 옛날 `platoon_fsm.py`(핸드셰이크 기반 JOIN, launch
인자 방식) 기준이라 지금 구조랑 안 맞음. 이 문서가 최신.

## 0. 공통 — 새 터미널 열 때마다 먼저

```bash
source ~/ros2_ws/install/setup.bash
```

코드를 새로 받았으면 재빌드 (메시지 필드도 바뀌었으면 platoon_interfaces도 같이):

```bash
cd ~/ros2_ws
colcon build --packages-select platoon_interfaces platoon
source install/setup.bash
```

## 1. 차량 켜기

```bash
ros2 launch platoon platoon.launch.py
```

**launch 인자는 안 먹는다** — `platoon.launch.py` 안에 `CAR_ID`/`VEHICLE_ID`/
`IS_DESIGNATED_LEADER`/`LEADER_ID`/`INITIAL_PARTNER_ID`/`EXPECTED_FOLLOWER_IDS`/
`INITIAL_LANE` 등이 차량별로 하드코딩돼 있음. 다른 차량용으로 쓰려면 그 파일
안 상수를 직접 고쳐야 함.

이 한 줄로 `lane_detector_node`(카메라·차선인식) / `decision_node`(자율주행·
차선변경) / `control_node`(STM32) / `fsm_decision_node`(플래툰 FSM) /
`v2x_node`(ESP32) 5개 노드가 한 번에 뜬다.

## 2. 출발/정지 (예전 "go"/"stop"에 해당)

```bash
ros2 param set /car1/decision_node is_running true    # 출발
ros2 param set /car1/decision_node is_running false   # 정지
```

`car1`을 해당 차량 namespace(`car1`/`car2`/`car3`)로 바꿔서 실행.

## 3. 플래툰 명령 (JOIN / EXIT / EXIT_TOGETHER)

```bash
# Platoon2에 JOIN — lane1로 차선변경하며 리더와 결합
ros2 topic pub --once /car2/platoon_cmd std_msgs/String "{data: 'JOIN'}"

# Platoon2 단독 EXIT — lane2로 차선변경 후 완전 이탈 (숫자는 목표 lane, 생략 시 기본 2)
ros2 topic pub --once /car2/platoon_cmd std_msgs/String "{data: 'EXIT:2'}"

# Platoon1·Platoon3 동시에 — 대형(연결) 유지한 채 차선변경만 (완전 해체 아님)
ros2 topic pub --once /car1/platoon_cmd std_msgs/String "{data: 'EXIT_TOGETHER:2'}"
ros2 topic pub --once /car3/platoon_cmd std_msgs/String "{data: 'EXIT_TOGETHER:2'}"
```

JOIN은 SOLO_DRIVE 상태의 팔로워에게만, EXIT/EXIT_TOGETHER는 PLATOON_MAINTAIN
상태에서만 먹는다 (다른 상태에서 보내면 무시됨).

## 4. 상태 확인

```bash
ros2 topic echo /car1/fsm_state_debug     # FSM 상태 | 모드 | 속도단계 | 현재 lane
ros2 topic echo /car1/v2x/targets         # 주변 차량 V2X 정보 (platoon_state/emergency/speed_level)
ros2 topic echo /car1/v2x/self_status     # 내가 방송 중인 상태
ros2 topic echo /car1/telemetry           # STM32 엔코더/초음파
ros2 topic echo /car1/vehicle_cmd         # decision_node가 실제로 내는 조향/speed_mode
ros2 topic list
ros2 topic hz /car1/lane_info
```

`fsm_state_debug` 정상 합류 시 흐름: `SOLO_DRIVE → PLATOON_JOIN → PLATOON_MAINTAIN`.

## 5. 차선변경 직접 테스트 (FSM 안 거치고 decision_node만)

```bash
ros2 service call /car1/request_lane_change std_srvs/srv/Trigger
```

## 6. 자율주행 PID 튜닝

```bash
ros2 param set /car1/decision_node kp_gain 0.15
ros2 param set /car1/decision_node kd_gain 0.05
ros2 param set /car1/decision_node ff_gain 10.0
```

`TURN_ANGLE`/`PHASE1_DIST`/`TARGET_CHANGE_DISTANCE`/`RELEASE_ANGLE`은 코드 상수라
파라미터로는 안 바뀜 — `decision_node.py` 소스 수정 후 재빌드 필요.

## 7. 떠있는 노드 확인 (문제 생겼을 때)

```bash
ros2 node list
```

`/car1/lane_detector_node`, `/car1/decision_node`, `/car1/control_node`,
`/car1/fsm_decision_node`, `/car1/v2x_node` 5개만 있어야 정상. 중복되거나
낯선 노드가 보이면 이전 실행이 안 죽고 남아있는 것:

```bash
ps aux | grep -E "decision_node|control_node|fsm_decision_node|lane_detector_node|v2x_node"
kill <PID>
ros2 daemon stop && ros2 daemon start
```
