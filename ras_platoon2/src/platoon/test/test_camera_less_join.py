import sys
import types
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
MODULE_ROOT = ROOT / "platoon" if (ROOT / "platoon").is_dir() else ROOT
package = types.ModuleType("platoon")
package.__path__ = [str(MODULE_ROOT)]
sys.modules.setdefault("platoon", package)

from platoon import platoon_fsm as fsm_module
from platoon.platoon_fsm import EgoState, NearbyVehicle, PlatoonFSM, PlatoonState
from platoon.v2x_protocol import LoopbackBus


class Clock:
    def __init__(self):
        self.now = 1000.0

    def time(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


def vehicle_view(vehicle_id, speed=0.4):
    return NearbyVehicle(
        vehicle_id=vehicle_id,
        checkpoint=0,
        speed=speed,
        heading=0.0,
        lane=0,
        platoon_allow=True,
        uwb_distance=0.5,
        uwb_angle=0.0,
    )


class CameraLessJoinTest(unittest.TestCase):
    def test_camera_less_follower_reaches_maintain(self):
        clock = Clock()
        with patch.object(fsm_module.time, "time", clock.time):
            bus = LoopbackBus()
            leader = PlatoonFSM(
                vehicle_id=101,
                comm=bus.create_port(101),
                is_designated_leader=True,
            )
            follower = PlatoonFSM(
                vehicle_id=102,
                comm=bus.create_port(102),
                is_designated_leader=False,
                allow_camera_less_join=True,
            )

            leader_ego = EgoState(
                destination=99,
                speed=0.4,
                heading=0.0,
                lane=0,
                lane_detected=True,
                lane_offset=0.0,
            )
            follower_ego = EgoState(
                destination=99,
                speed=0.4,
                heading=0.0,
                lane=0,
                lane_detected=False,
                lane_offset=0.0,
            )

            combined_targets = [vehicle_view(101), vehicle_view(102)]
            leader_nearby = combined_targets
            follower_nearby = combined_targets

            for _ in range(160):
                follower.update(follower_ego, follower_nearby)
                leader.update(leader_ego, leader_nearby)
                clock.advance(0.05)
                if (
                    follower.state == PlatoonState.PLATOON_MAINTAIN
                    and leader.state == PlatoonState.PLATOON_MAINTAIN
                ):
                    break

            self.assertEqual(follower.state, PlatoonState.PLATOON_MAINTAIN)
            self.assertEqual(leader.state, PlatoonState.PLATOON_MAINTAIN)

    def test_prefilter_excludes_own_vehicle_id(self):
        fsm = PlatoonFSM(vehicle_id=102, is_designated_leader=False)
        ego = EgoState(platoon_allow=True)
        candidates = fsm._prefilter(
            ego,
            [vehicle_view(102), vehicle_view(101)],
        )
        self.assertEqual([v.vehicle_id for v in candidates], [101])


if __name__ == "__main__":
    unittest.main()
