import unittest
from pathlib import Path


class P2LaunchProfileTest(unittest.TestCase):
    @staticmethod
    def launch_source():
        root = Path(__file__).resolve().parents[1]
        path = root / "launch" / "platoon.launch.py"
        if not path.exists():
            path = root / "platoon.launch.py"
        return path.read_text()

    def test_state_only_profile_disables_hardware_nodes_by_default(self):
        source = self.launch_source()
        self.assertIn("DeclareLaunchArgument('enable_control_node', default_value='false')", source)
        self.assertIn("DeclareLaunchArgument('enable_lane_detector', default_value='false')", source)
        self.assertIn("condition=IfCondition(LaunchConfiguration('enable_control_node'))", source)
        self.assertIn("condition=IfCondition(LaunchConfiguration('enable_lane_detector'))", source)

    def test_state_only_profile_enables_join_fallbacks_by_default(self):
        source = self.launch_source()
        self.assertIn("DeclareLaunchArgument('allow_camera_less_join', default_value='true')", source)
        self.assertIn("DeclareLaunchArgument('enable_ros_peer_fallback', default_value='true')", source)

    def test_demo_destination_prevents_immediate_exit(self):
        root = Path(__file__).resolve().parents[1]
        launch = self.launch_source()
        fsm_node_path = root / "platoon" / "fsm_decision_node.py"
        if not fsm_node_path.exists():
            fsm_node_path = root / "fsm_decision_node.py"
        fsm_node = fsm_node_path.read_text()
        self.assertIn("DeclareLaunchArgument('destination_id', default_value='9999')", launch)
        self.assertIn("self.ego_state.destination = self.destination_id", fsm_node)


if __name__ == "__main__":
    unittest.main()
