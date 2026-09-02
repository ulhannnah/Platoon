import unittest
from pathlib import Path


class DestinationWiringTest(unittest.TestCase):
    @staticmethod
    def package_file(relative_path, flat_name):
        root = Path(__file__).resolve().parents[1]
        path = root / relative_path
        if not path.exists():
            path = root / flat_name
        return path.read_text()

    def test_launch_exposes_nonzero_demo_destination(self):
        source = self.package_file("launch/platoon.launch.py", "platoon.launch.py")
        self.assertIn("DeclareLaunchArgument('destination_id', default_value='9999')", source)
        self.assertIn("'destination_id': ParameterValue(LaunchConfiguration('destination_id'), value_type=int)", source)

    def test_fsm_ego_uses_destination_parameter(self):
        source = self.package_file("platoon/fsm_decision_node.py", "fsm_decision_node.py")
        self.assertIn("self.ego_state.destination = self.destination_id", source)


if __name__ == "__main__":
    unittest.main()
