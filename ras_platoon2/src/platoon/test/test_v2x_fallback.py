import sys
import types
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MODULE_ROOT = ROOT / "platoon" if (ROOT / "platoon").is_dir() else ROOT
package = types.ModuleType("platoon")
package.__path__ = [str(MODULE_ROOT)]
sys.modules.setdefault("platoon", package)

from platoon.v2x_fallback import SelfStatusSnapshot, V2xFallbackTracker


def status(vehicle_id, leader_vehicle_id=0):
    return SelfStatusSnapshot(
        vehicle_id=vehicle_id,
        speed_mps=0.4,
        heading_deg=0.0,
        driving_state=2,
        platoon_state=1,
        platoon_id=0,
        platoon_enable=1,
        platoon_role=0,
        platoon_index=0,
        leader_vehicle_id=leader_vehicle_id,
        front_vehicle_id=0,
    )


class V2xFallbackTest(unittest.TestCase):
    def test_two_fresh_statuses_produce_synthetic_targets(self):
        tracker = V2xFallbackTracker(
            synthetic_distance_m=0.5,
            stale_after_s=1.0,
            real_silence_s=1.0,
        )
        tracker.observe(status(101), received_at=10.0)
        tracker.observe(status(102), received_at=10.0)

        targets = tracker.build_targets(now=10.1)

        self.assertEqual([target.vehicle_id for target in targets], [101, 102])
        self.assertTrue(all(target.distance_m == 0.5 for target in targets))
        self.assertTrue(all(target.uwb_valid == 0 for target in targets))
        self.assertTrue(all(target.espnow_valid == 0 for target in targets))
        self.assertTrue(all(target.leader_vehicle_id == -1 for target in targets))

    def test_recent_real_targets_suppress_fallback(self):
        tracker = V2xFallbackTracker(
            synthetic_distance_m=0.5,
            stale_after_s=2.0,
            real_silence_s=1.0,
        )
        tracker.observe(status(101), received_at=20.0)
        tracker.observe(status(102), received_at=20.0)
        tracker.note_real_targets(received_at=20.1)

        self.assertIsNone(tracker.build_targets(now=20.5))
        self.assertIsNotNone(tracker.build_targets(now=21.2))

    def test_one_or_stale_status_does_not_publish(self):
        tracker = V2xFallbackTracker(
            synthetic_distance_m=0.5,
            stale_after_s=1.0,
            real_silence_s=1.0,
        )
        tracker.observe(status(101), received_at=30.0)
        self.assertIsNone(tracker.build_targets(now=30.1))
        tracker.observe(status(102), received_at=30.0)
        self.assertIsNone(tracker.build_targets(now=31.1))


if __name__ == "__main__":
    unittest.main()
