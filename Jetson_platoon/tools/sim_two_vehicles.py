"""
sim_two_vehicles.py
실제 ESP32/차량 없이 여러 대의 PlatoonFSM을 한 프로세스에서 돌려
결합(JOIN) → 유지(MAINTAIN) → 해제(EXIT) 전 과정을 검증한다.

LoopbackBus가 ESP-NOW 브로드캐스트 역할을 대신하므로,
통신 하드웨어가 준비되기 전에도 로직 검증이 가능하다.

실차 시나리오를 그대로 따른다 — 리더는 젯슨 1대로 고정(is_designated_leader=True),
합류하는 차량은 전부 라즈베리파이(팔로워).

시나리오:
  A. 2대 결합      V1(젯슨/리더) ← V2 결합 → 목표거리 수렴 → MAINTAIN
  B. 3대 체인      V3이 맨 뒤(V2 뒤)로 합류. V3의 리더는 V2가 아니라 V1이어야 한다
  C. 정상 해제     공통 경로 끝에서 3대 모두 EXIT → 간격 확대 → SOLO 복귀
  D. 뒤차 소실     리더의 유일한 팔로워가 조용히 사라졌을 때 리더가 빠져나오는지

python3 tools/sim_two_vehicles.py 로 실행. 전부 통과하면 종료코드 0.
"""

import sys
import time
from pathlib import Path

# platoon_ws 안의 실제 코드(패키지)를 그대로 가져다 쓴다 — 사본을 따로 안 둔다.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "platoon_ws" / "src" / "platoon_control"))

from platoon_control.platoon_fsm import PlatoonFSM, EgoState, NearbyVehicle, PlatoonState
from platoon_control.v2x_protocol import LoopbackBus

ROUTE = [2, 3, 4, 5]
AHEAD_ANGLE = 0.02      # 상대가 내 앞 (UWB AoA 정면 부근)
BEHIND_ANGLE = 3.14     # 상대가 내 뒤


def make_view(v: "Vehicle", distance: float, angle: float) -> NearbyVehicle:
    """
    상대 차량이 V2X로 방송한 정보 + UWB 측정값을 합친 형태.

    ★ platoon_state / platoon_id / leader_id 를 반드시 실어보내야 한다.
      3대 이상 체인에서 신규 차량은 "내가 붙는 맨 뒤 차량이 알고 있는 진짜 리더 ID와
      플래툰 ID"를 이 필드들로 물려받는다(_setup_platoon). 이게 비어 있으면 3번째
      차량이 2번째 차량을 리더로 착각하고 별도 플래툰 ID를 새로 만들어버린다.
      → ESP32 V2X 브로드캐스트 패킷에 이 3개 필드가 들어가야 한다는 요구사항.
    """
    ego = v.ego
    return NearbyVehicle(
        vehicle_id=v.vid,
        checkpoint=ego.checkpoint,
        next_checkpoint=ego.next_checkpoint,
        route=list(ego.route),
        destination=ego.destination,
        lane=ego.lane,
        speed=ego.speed,
        heading=ego.heading,
        platoon_allow=ego.platoon_allow,
        emergency=ego.emergency,
        platoon_state=("PLATOON" if v.fsm.platoon_id is not None else "SOLO"),
        platoon_id=v.fsm.platoon_id,
        leader_id=v.fsm.leader_id,
        pdr=0.97,
        rssi=-52.0,
        uwb_distance=distance,
        uwb_angle=angle,
        timestamp=time.time(),
    )


class Vehicle:
    def __init__(self, vid: int, bus: LoopbackBus, is_leader: bool = False):
        self.vid = vid
        self.ego = EgoState(checkpoint=1, next_checkpoint=2, route=list(ROUTE),
                            destination=5, speed=0.5, heading=0.0, lane=0,
                            lane_detected=True, lane_offset=0.02)
        self.fsm = PlatoonFSM(vehicle_id=vid, comm=bus.create_port(vid),
                              is_designated_leader=is_leader)
        self.last_cmd = None


def step(convoy: list, gaps: list) -> None:
    """
    convoy는 앞에서 뒤 순서. gaps[i] = convoy[i]와 convoy[i+1] 사이 거리(m).
    각 차량이 나머지 차량을 UWB 거리·각도와 함께 보는 상황을 만들어 한 주기 돌린다.
    """
    pos = [0.0]
    for g in gaps:
        pos.append(pos[-1] + g)   # 뒤로 갈수록 값이 커진다

    for i, v in enumerate(convoy):
        seen = [make_view(o, abs(pos[i] - pos[j]),
                          AHEAD_ANGLE if pos[j] < pos[i] else BEHIND_ANGLE)
                for j, o in enumerate(convoy) if j != i]
        v.last_cmd = v.fsm.update(v.ego, seen)


def line(tag: str, convoy: list) -> None:
    print(f"  {tag:>12s} | " + " | ".join(
        f"V{v.vid} {v.fsm.state.name:16s} role={str(v.fsm.role):8s}"
        f" leader={str(v.fsm.leader_id):4s} 앞차={str(v.fsm.partner_id):4s}"
        f" 뒤차={str(v.fsm.successor_id):4s} pid={v.fsm.platoon_id}"
        for v in convoy))


def check(label: str, ok: bool, detail: str = "") -> bool:
    print(f"  [{'PASS' if ok else 'FAIL'}] {label}" + (f" — {detail}" if detail else ""))
    return ok


def header(title: str) -> None:
    print()
    print("=" * 78)
    print(f" {title}")
    print("=" * 78)


def drive_to_target(convoy: list, gaps_seq: list, done, limit_note: str = "") -> None:
    """목표거리까지 간격을 좁히며(또는 벌리며) 돌린다. done(convoy)가 True면 조기 종료."""
    for gaps in gaps_seq:
        step(convoy, gaps)
        print(f"  gaps={['%.2f' % g for g in gaps]} | " + " | ".join(
            f"V{v.vid} {v.fsm.state.name}"
            f"/{v.fsm.join_sub_state.name if v.fsm.join_sub_state else '-'}"
            for v in convoy))
        if done(convoy):
            return
        time.sleep(0.35)   # STABLE_TIME_S(1초) 판정을 위해 실제 시간 경과 필요


def all_in(convoy: list, state: PlatoonState) -> bool:
    return all(v.fsm.state == state for v in convoy)


# ══════════════════════════════════════════════════════════════════════
def scenario_chain() -> bool:
    """A~C: 2대 결합 → 3대 체인 → 정상 해제"""
    bus = LoopbackBus()
    v1 = Vehicle(1, bus, is_leader=True)    # 젯슨 — 항상 리더
    v2 = Vehicle(2, bus)                    # 라즈베리파이
    v3 = Vehicle(3, bus)                    # 라즈베리파이
    ok = True

    header("A. 2대 결합 — V2가 리더 V1에게 결합 요청")
    # V3은 아직 6m 뒤 (MAX_JOIN_DISTANCE_M=5.0 밖이라 후보에 안 잡힘)
    for cycle in range(40):
        step([v1, v2, v3], [1.5, 6.0])
        if v2.fsm.state == PlatoonState.PLATOON_JOIN:
            step([v1, v2, v3], [1.5, 6.0])   # V1이 SETUP을 받을 한 주기
            break
    line(f"cycle {cycle}", [v1, v2, v3])
    ok &= check("V2가 V1을 리더로 결합", v2.fsm.role == "FOLLOWER" and v2.fsm.leader_id == 1)
    ok &= check("V1이 리더 역할 채택", v1.fsm.role == "LEADER" and v1.fsm.successor_id == 2)
    ok &= check("V3은 아직 단독주행", v3.fsm.state == PlatoonState.SOLO_DRIVE)

    header("B-1. 목표거리 0.8m로 수렴 → MAINTAIN")
    drive_to_target([v1, v2, v3],
                    [[d, 6.0] for d in (1.5, 1.2, 1.0, 0.9, 0.85, 0.8, 0.8, 0.8, 0.8, 0.8)],
                    lambda c: c[0].fsm.state == PlatoonState.PLATOON_MAINTAIN
                              and c[1].fsm.state == PlatoonState.PLATOON_MAINTAIN)
    ok &= check("V1·V2 모두 MAINTAIN 진입",
                v1.fsm.state == PlatoonState.PLATOON_MAINTAIN
                and v2.fsm.state == PlatoonState.PLATOON_MAINTAIN)
    print(f"  공통 경로={v2.fsm.platoon_route}, EXIT 시작점={v2.fsm.exit_start_checkpoint}")

    header("B-2. 3대 체인 — V3이 맨 뒤(V2 뒤)로 합류")
    for cycle in range(60):
        step([v1, v2, v3], [0.8, 1.5])
        if v3.fsm.state == PlatoonState.PLATOON_JOIN:
            step([v1, v2, v3], [0.8, 1.5])   # V2가 SETUP을 받을 한 주기
            break
    line(f"cycle {cycle}", [v1, v2, v3])
    ok &= check("V3의 리더는 V2가 아니라 V1", v3.fsm.leader_id == 1,
                f"leader_id={v3.fsm.leader_id}")
    ok &= check("V3의 거리제어 대상(앞차)은 V2", v3.fsm.partner_id == 2,
                f"partner_id={v3.fsm.partner_id}")
    ok &= check("플래툰 ID를 3대가 공유",
                v1.fsm.platoon_id == v2.fsm.platoon_id == v3.fsm.platoon_id,
                f"{v1.fsm.platoon_id}/{v2.fsm.platoon_id}/{v3.fsm.platoon_id}")
    ok &= check("V2의 맨 뒤 자리가 닫힘", v2.fsm.successor_id == 3)
    ok &= check("V1은 여전히 V2를 앞차로 붙들고 있음",
                v1.fsm.partner_id == 2 and v1.fsm.successor_id == 2)

    drive_to_target([v1, v2, v3],
                    [[0.8, d] for d in (1.5, 1.2, 1.0, 0.9, 0.85, 0.8, 0.8, 0.8, 0.8, 0.8)],
                    lambda c: c[2].fsm.state == PlatoonState.PLATOON_MAINTAIN)
    ok &= check("V3도 MAINTAIN 진입", v3.fsm.state == PlatoonState.PLATOON_MAINTAIN)

    header("C. 정상 해제 — 공통 경로 끝에서 3대 모두 EXIT")
    for cp in (2, 3, 4):
        for v in (v1, v2, v3):
            v.ego.checkpoint = cp
        step([v1, v2, v3], [0.8, 0.8])
        print(f"  CP{cp} | " + " | ".join(f"V{v.vid} {v.fsm.state.name}" for v in (v1, v2, v3)))
    ok &= check("3대 모두 EXIT 진입", all_in([v1, v2, v3], PlatoonState.PLATOON_EXIT))

    drive_to_target([v1, v2, v3],
                    [[d, d] for d in (1.2, 1.7, 2.1, 2.1, 2.1, 2.1, 2.1)],
                    lambda c: all_in(c, PlatoonState.SOLO_DRIVE))
    ok &= check("3대 모두 SOLO_DRIVE 복귀", all_in([v1, v2, v3], PlatoonState.SOLO_DRIVE))
    ok &= check("플래툰 정보 초기화",
                all(v.fsm.platoon_id is None and v.fsm.role is None
                    and v.fsm.partner_id is None and v.fsm.successor_id is None
                    for v in (v1, v2, v3)))
    return ok


def form_two_car_platoon() -> tuple:
    """V1(젯슨 리더) + V2(팔로워)를 MAINTAIN까지 끌고 간다. 다른 시나리오의 준비 단계."""
    bus = LoopbackBus()
    v1 = Vehicle(1, bus, is_leader=True)
    v2 = Vehicle(2, bus)

    for _ in range(40):
        step([v1, v2], [1.5])
        if v2.fsm.state == PlatoonState.PLATOON_JOIN:
            step([v1, v2], [1.5])
            break
    drive_to_target([v1, v2],
                    [[d] for d in (1.5, 1.2, 1.0, 0.9, 0.85, 0.8, 0.8, 0.8, 0.8, 0.8)],
                    lambda c: all_in(c, PlatoonState.PLATOON_MAINTAIN))
    return v1, v2


def scenario_leader_loses_follower() -> bool:
    """
    D. 리더의 유일한 팔로워가 LEAVE도 못 보내고 사라지는 경우
       (전원 차단·통신 범위 이탈). 리더가 사라진 차를 붙든 채
       MAINTAIN에 갇히면 안 된다.
    """
    header("D. 뒤차 소실 — 리더가 빈 플래툰에 갇히지 않는지")
    v1, v2 = form_two_car_platoon()
    ok = check("사전조건: 2대 MAINTAIN", all_in([v1, v2], PlatoonState.PLATOON_MAINTAIN))

    # V2가 V2X/UWB 목록에서 사라진다 (LEAVE 패킷 없음)
    v1.fsm.update(v1.ego, [])
    print("  → V2 소실. PARTNER_LOST_TIMEOUT_S(1.0초) 경과 대기")
    time.sleep(1.1)
    v1.fsm.update(v1.ego, [])
    line("소실 후", [v1])
    ok &= check("리더가 SOLO_DRIVE로 복귀", v1.fsm.state == PlatoonState.SOLO_DRIVE)
    ok &= check("사라진 차를 앞차로 붙들고 있지 않음",
                v1.fsm.partner_id is None and v1.fsm.successor_id is None)
    return ok


def scenario_no_leader() -> bool:
    """
    E. 젯슨 없이 라즈베리파이 2대만 있는 경우.
       실차 시나리오상 리더는 젯슨 1대 고정이므로, 팔로워끼리는 플래툰을
       만들면 안 된다 (요청을 받아준 쪽이 리더가 되어버림).
    """
    header("E. 리더 없는 조합 — 팔로워끼리는 결합하지 않아야 함")
    bus = LoopbackBus()
    a = Vehicle(7, bus)
    b = Vehicle(8, bus)

    for _ in range(60):
        step([a, b], [1.2])
    line("60주기 후", [a, b])
    return check("둘 다 단독주행 유지",
                 all_in([a, b], PlatoonState.SOLO_DRIVE)
                 and a.fsm.role is None and b.fsm.role is None)


def scenario_leader_front_obstacle() -> bool:
    """
    F. 리더의 전방 장애물 판정.
       리더는 partner_id가 '뒤차'라서, 초음파-UWB 교차판정을 뒤차 거리에
       적용하면 진짜 전방 장애물을 "앞차 자신"으로 오판해 비상정지를 놓친다.

       억제가 실제로 일어나는 구간을 골라야 이 테스트가 의미가 있다.
       뒤차가 0.40m까지 바짝 붙은 상태(팔로워 오버슛)에서 전방 0.28m 장애물이면
         d(0.28) >= 뒤차거리(0.40) - CUTIN_MARGIN_M(0.15) = 0.25  → 참
       이라서 수정 전 코드는 "초음파가 본 게 앞차"라며 아무 반응도 하지 않는다.
    """
    header("F. 리더 전방 장애물 — 뒤차 거리로 오판하지 않는지")
    v1, v2 = form_two_car_platoon()
    ok = check("사전조건: 2대 MAINTAIN", all_in([v1, v2], PlatoonState.PLATOON_MAINTAIN))

    v1.ego.front_distance = 0.28          # OBSTACLE_STOP_DISTANCE_M(0.30) 이내
    step([v1, v2], [0.40])                # 뒤차 V2가 0.40m까지 붙은 상태
    print(f"  리더 초음파 0.28m / 뒤차 UWB 0.40m → cmd={v1.last_cmd}")
    ok &= check("리더가 비상정지", v1.last_cmd.emergency is True
                and v1.last_cmd.target_speed == 0.0,
                f"reason={v1.fsm.emergency_reason}")
    return ok


def run() -> int:
    results = {
        "A~C 3대 체인 결합·해제": scenario_chain(),
        "D 뒤차 소실 복구": scenario_leader_loses_follower(),
        "E 리더 없는 조합 차단": scenario_no_leader(),
        "F 리더 전방 장애물 판정": scenario_leader_front_obstacle(),
    }
    header("결과 요약")
    for name, ok in results.items():
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}")
    return 0 if all(results.values()) else 1


if __name__ == "__main__":
    sys.exit(run())
