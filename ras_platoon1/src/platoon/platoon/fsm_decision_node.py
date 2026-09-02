"""
FSM Decision Node wrapper for PlatoonFSM.

This module tries to run as a ROS2 node using `rclpy`. If `rclpy` is not
available, it falls back to a standalone runner that exercises the FSM with
the same example used in `platoon_fsm.py`'s __main__ section.

Topics (JSON, simple schema):
- Subscribes to: `ego_json` (std_msgs/String) — JSON of EgoState fields
- Subscribes to: `nearby_json` (std_msgs/String) — JSON list of NearbyVehicle dicts
- Publishes to:  `driving_command_json` (std_msgs/String) — JSON of DrivingCommand

This lightweight wrapper avoids custom ROS messages to make running easier
during development.
"""
from __future__ import annotations
import json
import time
from typing import List

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    RCLPY_AVAILABLE = True
except Exception:
    RCLPY_AVAILABLE = False

from .platoon_fsm import PlatoonFSM, EgoState, NearbyVehicle, DrivingCommand


def ego_from_dict(d: dict) -> EgoState:
    # Only set known fields; extra keys ignored.
    return EgoState(
        checkpoint=d.get("checkpoint", 0),
        next_checkpoint=d.get("next_checkpoint", 0),
        route=d.get("route", []),
        destination=d.get("destination", 0),
        speed=d.get("speed", 0.0),
        accel=d.get("accel", 0.0),
        heading=d.get("heading", 0.0),
        lane=d.get("lane", 0),
        lane_offset=d.get("lane_offset", 0.0),
        lane_detected=d.get("lane_detected", False),
        obstacle=d.get("obstacle", False),
        front_distance=d.get("front_distance", None),
        stm32_failsafe=d.get("stm32_failsafe", False),
        platoon_allow=d.get("platoon_allow", True),
        emergency=d.get("emergency", False),
    )


def nearby_from_list(lst: List[dict]) -> List[NearbyVehicle]:
    out = []
    for d in lst:
        nv = NearbyVehicle(
            vehicle_id=d.get("vehicle_id", 0),
            checkpoint=d.get("checkpoint", 0),
            next_checkpoint=d.get("next_checkpoint", 0),
            route=d.get("route", []),
            destination=d.get("destination", 0),
            lane=d.get("lane", 0),
            speed=d.get("speed", 0.0),
            accel=d.get("accel", 0.0),
            heading=d.get("heading", 0.0),
            platoon_allow=d.get("platoon_allow", True),
            platoon_state=d.get("platoon_state", "SOLO"),
            platoon_id=d.get("platoon_id", None),
            leader_id=d.get("leader_id", None),
            pdr=d.get("pdr", 1.0),
            rssi=d.get("rssi", 0.0),
            emergency=d.get("emergency", False),
            uwb_distance=d.get("uwb_distance", None),
            uwb_angle=d.get("uwb_angle", None),
            timestamp=d.get("timestamp", time.time()),
        )
        out.append(nv)
    return out


def driving_command_to_json(cmd: DrivingCommand) -> str:
    return json.dumps({
        "mode": cmd.mode,
        "target_lane": cmd.target_lane,
        "target_speed": cmd.target_speed,
        "target_distance": cmd.target_distance,
        "leader_id": cmd.leader_id,
        "accel_request": cmd.accel_request,
        "emergency": cmd.emergency,
    })


if RCLPY_AVAILABLE:
    class FSMDecisionNode(Node):
        def __init__(self, node_name: str = "fsm_decision_node", vehicle_id: int = 1):
            super().__init__(node_name)
            self.get_logger().info("Starting FSMDecisionNode")
            self.fsm = PlatoonFSM(vehicle_id=vehicle_id)
            self._ego = EgoState()
            self._nearby: List[NearbyVehicle] = []

            self.sub_ego = self.create_subscription(String, "ego_json", self._on_ego, 10)
            self.sub_nearby = self.create_subscription(String, "nearby_json", self._on_nearby, 10)
            self.pub_cmd = self.create_publisher(String, "driving_command_json", 10)

            self.create_timer(0.1, self._timer_cb)  # 10 Hz

        def _on_ego(self, msg: String) -> None:
            try:
                d = json.loads(msg.data)
                self._ego = ego_from_dict(d)
            except Exception as e:
                self.get_logger().warn(f"Invalid ego_json: {e}")

        def _on_nearby(self, msg: String) -> None:
            try:
                lst = json.loads(msg.data)
                self._nearby = nearby_from_list(lst)
            except Exception as e:
                self.get_logger().warn(f"Invalid nearby_json: {e}")

        def _timer_cb(self) -> None:
            cmd = self.fsm.update(self._ego, self._nearby)
            out = driving_command_to_json(cmd)
            self.pub_cmd.publish(String(data=out))


def main_ros2():
    rclpy.init()
    node = FSMDecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main_standalone():
    # Fallback runner that mimics the example in platoon_fsm.py
    class PrintComm:
        def send(self, packet) -> None:
            print(f"  [SEND] {packet}")

    fsm = PlatoonFSM(vehicle_id=1, comm=PrintComm())
    ego = EgoState(checkpoint=1, destination=5, speed=0.5, heading=0.0)
    candidate = NearbyVehicle'
        vehicle_id=2, checkpoint=1, speed=0.5, heading=0.02,
        uwb_distance=1.2, uwb_angle=0.1,
    )

    for i in range(12):
        cmd = fsm.update(ego, [candidate])
        print(f"cycle {i}: {fsm.state} / {fsm.match_state} / {cmd}")
        time.sleep(0.1)


if __name__ == "__main__":
    if RCLPY_AVAILABLE:
        main_ros2()
    else:
        print("rclpy not available — running standalone example")
        main_standalone()
