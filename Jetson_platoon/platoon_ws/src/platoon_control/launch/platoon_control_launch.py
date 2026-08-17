"""
platoon_control_launch.py
mode, is_leader 파라미터를 launch 인자로 넘길 수 있는 런치 파일.

v2x_node(ESP32 통신)와 platoon_control_node(FSM+제어)를 함께 띄운다 — FSM 노드가
/v2x/esp32_data, /v2x/outgoing, /v2x/incoming 토픽으로 v2x_node와 통신하므로
(ros_v2x_comm.py 참고) 둘 다 떠 있어야 정상 동작한다.

사용:
    # 젯슨(리더) 차량
    ros2 launch platoon_control platoon_control_launch.py is_leader:=true

    # 라즈베리파이(팔로워) 차량 — is_leader 생략 시 기본값 false
    ros2 launch platoon_control platoon_control_launch.py mode:=manual
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mode_arg = DeclareLaunchArgument(
        "mode", default_value="auto", description="시작 모드: auto 또는 manual"
    )
    is_leader_arg = DeclareLaunchArgument(
        "is_leader", default_value="false",
        description="true면 이 차량은 항상 리더(젯슨). false면 항상 팔로워(라즈베리파이)",
    )

    v2x_node = Node(
        package="platoon_control",
        executable="v2x_node",
        name="v2x_node",
        output="screen",
    )

    fsm_node = Node(
        package="platoon_control",
        executable="platoon_control_node",
        name="platoon_control_node",
        output="screen",
        parameters=[{
            "mode": LaunchConfiguration("mode"),
            "is_leader": LaunchConfiguration("is_leader"),
        }],
    )

    return LaunchDescription([mode_arg, is_leader_arg, v2x_node, fsm_node])