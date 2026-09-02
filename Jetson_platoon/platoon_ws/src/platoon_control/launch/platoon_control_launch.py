"""
platoon_control_launch.py
mode, is_leader 파라미터를 launch 인자로 넘길 수 있는 런치 파일.

v2x_node(ESP32 통신), lane_detector_node(카메라+차선인식), platoon_control_node
(FSM+제어)를 함께 띄운다. platoon_control_node는 /v2x/esp32_data·/v2x/outgoing·
/v2x/incoming으로 v2x_node와, /lane_info로 lane_detector_node와 통신하므로
셋 다 떠 있어야 정상 동작한다.

사용:
    # 젯슨(리더) 차량
    ros2 launch platoon_control platoon_control_launch.py is_leader:=true

    # 라즈베리파이(팔로워) 차량 — is_leader 생략 시 기본값 false
    ros2 launch platoon_control platoon_control_launch.py mode:=manual \
        camera_type:=picamera2 calibration_file:=/home/pi/camera_params.npz
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
    camera_type_arg = DeclareLaunchArgument(
        "camera_type", default_value="auto",
        description="auto(기본)/picamera2/usb/csi. 라즈베리파이는 auto로 두면 picamera2가 우선 시도됨",
    )
    calibration_file_arg = DeclareLaunchArgument(
        "calibration_file", default_value="",
        description="렌즈 왜곡보정 npz 경로. 비워두면 보정 없이 진행",
    )
    lane_image_width_arg = DeclareLaunchArgument(
        "lane_image_width", default_value="640",
        description="lane_detector_node의 캡처 폭(px)과 반드시 일치해야 함 — "
                    "platoon_control_node가 LaneInfo.offset(픽셀)을 정규화할 때 씀",
    )

    v2x_node = Node(
        package="platoon_control",
        executable="v2x_node",
        name="v2x_node",
        output="screen",
    )

    lane_detector_node = Node(
        package="platoon_control",
        executable="lane_detector_node",
        name="lane_detector_node",
        output="screen",
        parameters=[{
            "camera_type": LaunchConfiguration("camera_type"),
            "calibration_file": LaunchConfiguration("calibration_file"),
        }],
    )

    fsm_node = Node(
        package="platoon_control",
        executable="platoon_control_node",
        name="platoon_control_node",
        output="screen",
        parameters=[{
            "mode": LaunchConfiguration("mode"),
            "is_leader": LaunchConfiguration("is_leader"),
            "lane_image_width": LaunchConfiguration("lane_image_width"),
        }],
    )

    return LaunchDescription([
        mode_arg, is_leader_arg, camera_type_arg, calibration_file_arg, lane_image_width_arg,
        v2x_node, lane_detector_node, fsm_node,
    ])
