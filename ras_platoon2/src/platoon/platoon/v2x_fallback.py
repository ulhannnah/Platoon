"""Temporary ROS self-status fallback for two-vehicle V2X demos.

This module is deliberately independent of rclpy so the arbitration policy can
be tested without ROS. Synthetic targets are marked as non-UWB/non-ESP-NOW and
must never override recently received real ESP32 target frames.
"""
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple


@dataclass(frozen=True)
class SelfStatusSnapshot:
    vehicle_id: int
    speed_mps: float
    heading_deg: float
    driving_state: int
    platoon_state: int
    platoon_id: int
    platoon_enable: int
    platoon_role: int
    platoon_index: int
    leader_vehicle_id: int
    front_vehicle_id: int


@dataclass(frozen=True)
class FallbackTargetSnapshot:
    vehicle_id: int
    distance_m: float
    angle_deg: float
    rel_x_m: float
    rel_y_m: float
    speed_mps: float
    heading_deg: float
    driving_state: int
    platoon_state: int
    platoon_id: int
    platoon_enable: int
    platoon_role: int
    platoon_index: int
    uwb_valid: int
    espnow_valid: int
    confidence: float
    leader_vehicle_id: int
    front_vehicle_id: int


class V2xFallbackTracker:
    """Select real V2X when available; otherwise expose fresh ROS peer states."""

    def __init__(
        self,
        synthetic_distance_m: float = 0.5,
        stale_after_s: float = 1.0,
        real_silence_s: float = 1.0,
    ):
        self.synthetic_distance_m = float(synthetic_distance_m)
        self.stale_after_s = float(stale_after_s)
        self.real_silence_s = float(real_silence_s)
        self._statuses: Dict[int, Tuple[float, SelfStatusSnapshot]] = {}
        self._last_real_targets_at: Optional[float] = None

    def observe(self, status: SelfStatusSnapshot, received_at: float) -> None:
        if status.vehicle_id <= 0:
            return
        self._statuses[status.vehicle_id] = (float(received_at), status)

    def note_real_targets(self, received_at: float) -> None:
        self._last_real_targets_at = float(received_at)

    def build_targets(self, now: float) -> Optional[List[FallbackTargetSnapshot]]:
        now = float(now)
        if (
            self._last_real_targets_at is not None
            and now - self._last_real_targets_at <= self.real_silence_s
        ):
            return None

        fresh = [
            status
            for received_at, status in self._statuses.values()
            if now - received_at <= self.stale_after_s
        ]
        if len(fresh) < 2:
            return None

        return [self._to_target(status) for status in sorted(fresh, key=lambda item: item.vehicle_id)]

    def _to_target(self, status: SelfStatusSnapshot) -> FallbackTargetSnapshot:
        leader_vehicle_id = status.leader_vehicle_id if status.leader_vehicle_id > 0 else -1
        return FallbackTargetSnapshot(
            vehicle_id=status.vehicle_id,
            distance_m=self.synthetic_distance_m,
            angle_deg=0.0,
            rel_x_m=self.synthetic_distance_m,
            rel_y_m=0.0,
            speed_mps=status.speed_mps,
            heading_deg=status.heading_deg,
            driving_state=status.driving_state,
            platoon_state=status.platoon_state,
            platoon_id=status.platoon_id,
            platoon_enable=status.platoon_enable,
            platoon_role=status.platoon_role,
            platoon_index=status.platoon_index,
            uwb_valid=0,
            espnow_valid=0,
            confidence=0.0,
            leader_vehicle_id=leader_vehicle_id,
            front_vehicle_id=status.front_vehicle_id,
        )
