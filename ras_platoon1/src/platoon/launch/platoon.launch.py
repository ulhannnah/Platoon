import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 1. 패키지 내 config 폴더에 있는 yaml 파일 경로를 가져옵니다.
    config_file = os.path.expanduser('~/ros2_ws/src/platoon/config/platoon_params.yaml')

    # 2. 카메라 인지 노드
    lane_detector = Node(
        package='platoon',
        executable='lane_detector_node',
        name='lane_detector_node',
        output='screen',
        emulate_tty=True,
        parameters=[config_file]

    )

    # 3. 플래툰 할때는 이 코드로
    '''
    fsm_decision = Node(
        package='platoon',
        executable='fsm_decision_node',
        name='fsm_decision_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'vehicle_id': 1,
            'is_designated_leader': False,
        }]
    )
    '''

    # 3-1. 단독 주행 판단 노드
    decision = Node(
        package='platoon',
        executable='decision_node',
        name='decision_node',
        output='screen',
        emulate_tty=True,
        parameters=[config_file]
    )

    # 4. STM32 제어 / UART 통신 노드
    control = Node(
        package='platoon',
        executable='control_node',
        name='control_node',
        output='screen',
        emulate_tty=True,
        parameters=[config_file]
    )

    return LaunchDescription([
        lane_detector,
        decision,
        control
    ])