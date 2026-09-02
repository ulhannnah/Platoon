import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # config 폴더의 yaml 파일 경로 (control_node / lane_detector_node 튜닝값)
    config_file = os.path.expanduser('~/ros2_ws/src/platoon/config/platoon_params.yaml')

    # 리더/팔로워 구분값 — 실행할 때 오버라이드
    #   리더:   ros2 launch platoon platoon.launch.py
    #   팔로워: ros2 launch platoon platoon.launch.py vehicle_id:=102 is_designated_leader:=false
    vehicle_id_arg = DeclareLaunchArgument('vehicle_id', default_value='101')
    is_leader_arg = DeclareLaunchArgument('is_designated_leader', default_value='true')
    destination_arg = DeclareLaunchArgument('destination_id', default_value='9999')
    camera_less_arg = DeclareLaunchArgument('allow_camera_less_join', default_value='true')
    control_node_arg = DeclareLaunchArgument('enable_control_node', default_value='false')
    lane_detector_arg = DeclareLaunchArgument('enable_lane_detector', default_value='false')
    # 기본은 false — V2X는 ESP32(UWB+ESP-NOW)로만 한다. 이 폴백을 켜면 ESP32가
    # 실제 타깃을 못 줄 때 같은 ROS 네트워크의 상대 차량 /v2x/self_status를 직접
    # 봐서 가짜 타깃을 합성하는데, 그러려면 두 차량이 ROS_DOMAIN_ID를 공유해야
    # 해서 /vehicle_cmd 등 다른 토픽까지 같이 새는 위험이 있다 (모터 연결 전까지
    # 보류). 데모에서 의도적으로 필요할 때만 true로 켤 것.
    v2x_fallback_arg = DeclareLaunchArgument('enable_ros_peer_fallback', default_value='false')

    # STM32 제어 / UART 통신 노드 (포트 자동탐색)
    control = Node(
        package='platoon',
        executable='control_node',
        name='control_node',
        output='screen',
        emulate_tty=True,
        parameters=[config_file],
        condition=IfCondition(LaunchConfiguration('enable_control_node')),
    )

    # ESP32-S3 V2X 통신 노드 (포트 자동탐색)
    v2x = Node(
        package='platoon',
        executable='v2x_node',
        name='v2x_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'vehicle_id': ParameterValue(LaunchConfiguration('vehicle_id'), value_type=int),
            'enable_ros_peer_fallback': ParameterValue(
                LaunchConfiguration('enable_ros_peer_fallback'), value_type=bool
            ),
        }],
    )

    # 플래툰 판단(FSM) 노드
    fsm_decision = Node(
        package='platoon',
        executable='fsm_decision_node',
        name='fsm_decision_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'vehicle_id': ParameterValue(LaunchConfiguration('vehicle_id'), value_type=int),
            'is_designated_leader': ParameterValue(LaunchConfiguration('is_designated_leader'), value_type=bool),
            'destination_id': ParameterValue(LaunchConfiguration('destination_id'), value_type=int),
            'allow_camera_less_join': ParameterValue(
                LaunchConfiguration('allow_camera_less_join'), value_type=bool
            ),
        }],
    )

    # 카메라 + 차선인식 노드
    lane_detector = Node(
        package='platoon',
        executable='lane_detector_node',
        name='lane_detector_node',
        output='screen',
        emulate_tty=True,
        parameters=[config_file],
        condition=IfCondition(LaunchConfiguration('enable_lane_detector')),
    )

    return LaunchDescription([
        vehicle_id_arg,
        is_leader_arg,
        destination_arg,
        camera_less_arg,
        control_node_arg,
        lane_detector_arg,
        v2x_fallback_arg,
        control,
        v2x,
        fsm_decision,
        lane_detector,
    ])
