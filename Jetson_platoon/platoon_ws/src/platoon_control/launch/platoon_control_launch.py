"""
platoon_control_launch.py
mode 파라미터를 launch 인자로 넘길 수 있는 런치 파일.

사용:
    ros2 launch platoon_control platoon_control_launch.py mode:=manual
    ros2 launch platoon_control platoon_control_launch.py mode:=auto   (기본값)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mode_arg = DeclareLaunchArgument(
        "mode", default_value="auto", description="시작 모드: auto 또는 manual"
    )

    node = Node(
        package="platoon_control",
        executable="platoon_control_node",
        name="platoon_control_node",
        output="screen",
        parameters=[{"mode": LaunchConfiguration("mode")}],
    )

    return LaunchDescription([mode_arg, node])
