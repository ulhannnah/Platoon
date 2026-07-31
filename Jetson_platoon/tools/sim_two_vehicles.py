"""
sim_two_vehicles.py
실제 ESP32/차량 없이 두 대의 PlatoonFSM을 한 프로세스에서 돌려
결합(JOIN) → 유지(MAINTAIN) → 해제(EXIT) 전 과정을 검증한다.

LoopbackBus가 ESP-NOW 브로드캐스트 역할을 대신하므로,
통신 하드웨어가 준비되기 전에도 로직 검증이 가능하다.

시나리오:
  차량 1 (뒤쪽) — 차량 2를 발견하고 결합 요청
  차량 2 (앞쪽) — 요청을 재검사하고 승인 → 리더가 됨
  이후 거리를 좁혀 결합하고, 공통 경로 끝에서 해제
"""

import sys
import time
from pathlib import Path

# platoon_ws 안의 실제 코드(패키지)를 그대로 가져다 쓴다 — 사본을 따로 안 둔다.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "platoon_ws" / "src" / "platoon_control"))

from platoon_control.platoon_fsm import PlatoonFSM, EgoState, NearbyVehicle, PlatoonState
from platoon_control.v2x_protocol import LoopbackBus


def make_view(of_vehicle: "Vehicle", distance: float, angle: float) -> NearbyVehicle:
    """상대 차량이 V2X로 방송한 정보 + UWB 측정값을 합친 형태"""
    ego = of_vehicle.ego
    return NearbyVehicle(
        vehicle_id=of_vehicle.vid,
        checkpoint=ego.checkpoint,
        next_checkpoint=ego.next_checkpoint,
        route=list(ego.route),
        destination=ego.destination,
        lane=ego.lane,
        speed=ego.speed,
        heading=ego.heading,
        platoon_allow=ego.platoon_allow,
        emergency=ego.emergency,
        pdr=0.97,
        rssi=-52.0,
        uwb_distance=distance,
        uwb_angle=angle,
        timestamp=time.time(),
    )


class Vehicle:
    def __init__(self, vid: int, bus: LoopbackBus, ego: EgoState):
        self.vid = vid
        self.ego = ego
        self.fsm = PlatoonFSM(vehicle_id=vid, comm=bus.create_port(vid))
        self.last_cmd = None


def run():
    bus = LoopbackBus()

    # 두 대 모두 같은 경로 [2,3,4,5]를 달리는 중, 같은 차선
    # lane_detected/lane_offset = 라인트레이싱이 차선 중앙을 잘 잡고 있는 상태
    v1 = Vehicle(1, bus, EgoState(checkpoint=1, next_checkpoint=2, route=[2, 3, 4, 5],
                                  destination=5, speed=0.5, heading=0.0, lane=0,
                                  lane_detected=True, lane_offset=0.02))
    v2 = Vehicle(2, bus, EgoState(checkpoint=1, next_checkpoint=2, route=[2, 3, 4, 5],
                                  destination=5, speed=0.5, heading=0.01, lane=0,
                                  lane_detected=True, lane_offset=0.03))

    # 차량 2가 차량 1보다 앞쪽 → UWB 각도 0 근처(정면), 차량 1 기준
    distance = 1.5
    angle = 0.03

    print("=" * 66)
    print(" 1단계: 매칭 및 결합 요청")
    print("=" * 66)

    for cycle in range(40):
        # 각 차량이 보는 상대방 (앞/뒤 관계에 따라 각도 부호가 반대)
        v1_sees = [make_view(v2, distance, angle)]
        v2_sees = [make_view(v1, distance, angle + 3.14159)]  # 뒤쪽 = 각도 180도 부근

        v1.last_cmd = v1.fsm.update(v1.ego, v1_sees)
        v2.last_cmd = v2.fsm.update(v2.ego, v2_sees)

        if cycle % 8 == 0 or v1.fsm.state != PlatoonState.SOLO_DRIVE:
            print(f"  [{cycle:2d}] V1: {v1.fsm.state.name:16s}/{v1.fsm.match_state.name:14s}"
                  f" | V2: {v2.fsm.state.name:16s}/{v2.fsm.match_state.name}")

        if v1.fsm.state == PlatoonState.PLATOON_JOIN and v2.fsm.state == PlatoonState.PLATOON_JOIN:
            break

    print()
    print(f"  결과 → V1: role={v1.fsm.role}, leader={v1.fsm.leader_id}, platoon={v1.fsm.platoon_id}")
    print(f"         V2: role={v2.fsm.role}, leader={v2.fsm.leader_id}, platoon={v2.fsm.platoon_id}")

    if v1.fsm.role is None or v2.fsm.role is None:
        print("\n  [실패] 결합이 성립하지 않음")
        return

    print()
    print("=" * 66)
    print(" 2단계: 거리 좁히며 결합 (목표 0.8m)")
    print("=" * 66)

    for distance in [1.5, 1.3, 1.1, 0.95, 0.85, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8]:
        v1_sees = [make_view(v2, distance, 0.02)]
        v2_sees = [make_view(v1, distance, 3.14159)]

        v1.last_cmd = v1.fsm.update(v1.ego, v1_sees)
        v2.last_cmd = v2.fsm.update(v2.ego, v2_sees)

        print(f"  d={distance:.2f}m | V1 {v1.fsm.state.name}"
              f"/{v1.fsm.join_sub_state.name if v1.fsm.join_sub_state else '-':16s}"
              f" speed={v1.last_cmd.target_speed:.3f}"
              f" | V2 {v2.fsm.state.name}"
              f"/{v2.fsm.join_sub_state.name if v2.fsm.join_sub_state else '-'}")

        if (v1.fsm.state == PlatoonState.PLATOON_MAINTAIN
                and v2.fsm.state == PlatoonState.PLATOON_MAINTAIN):
            break
        time.sleep(0.35)   # STABLE_TIME_S(1초) 판정을 위해 실제 시간 경과 필요

    print()
    print(f"  공통 경로: V1={v1.fsm.platoon_route}, EXIT 시작점={v1.fsm.exit_start_checkpoint}")

    print()
    print("=" * 66)
    print(" 3단계: 체크포인트 진행 및 해제")
    print("=" * 66)

    for cp in [2, 3, 4]:
        v1.ego.checkpoint = cp
        v2.ego.checkpoint = cp
        v1_sees = [make_view(v2, 0.8, 0.02)]
        v2_sees = [make_view(v1, 0.8, 3.14159)]
        v1.fsm.update(v1.ego, v1_sees)
        v2.fsm.update(v2.ego, v2_sees)
        print(f"  CP{cp} | V1 {v1.fsm.state.name} | V2 {v2.fsm.state.name}")

    for distance in [1.0, 1.5, 2.1, 2.1, 2.1, 2.1]:
        v1_sees = [make_view(v2, distance, 0.02)]
        v2_sees = [make_view(v1, distance, 3.14159)]
        v1.fsm.update(v1.ego, v1_sees)
        v2.fsm.update(v2.ego, v2_sees)
        print(f"  d={distance:.2f}m | V1 {v1.fsm.state.name} | V2 {v2.fsm.state.name}")
        if (v1.fsm.state == PlatoonState.SOLO_DRIVE
                and v2.fsm.state == PlatoonState.SOLO_DRIVE):
            break
        time.sleep(0.35)

    print()
    print("=" * 66)
    print(f" 최종: V1={v1.fsm.state.name}, V2={v2.fsm.state.name}")
    print(f"       플래툰 정보 초기화됨: V1 platoon={v1.fsm.platoon_id}, role={v1.fsm.role}")
    print("=" * 66)


if __name__ == "__main__":
    run()
