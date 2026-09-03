import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # --- [차량 1호기 설정] ---
    CAR_ID = 'car1'
    VEHICLE_ID = 101
    IS_DESIGNATED_LEADER = False
    DESTINATION_ID = 9999
    # -------------------------

    pkg_share = FindPackageShare('platoon').find('platoon')
    config_file = os.path.join(pkg_share, 'config', 'platoon_params.yaml')

    # 카메라 + 차선인식 노드
    lane_detector = Node(
        package='platoon',
        executable='lane_detector_node',
        name='lane_detector_node',
        namespace=CAR_ID,
        output='screen',
        emulate_tty=True,
        parameters=[config_file]
    )

    # 단독주행 노드
    decision = Node(
        package='platoon',
        executable='decision_node',
        name='decision_node',
        namespace=CAR_ID,
        output='screen',
        emulate_tty=True,
        parameters=[config_file]
    )

    # STM32 제어 / UART 통신 노드
    control = Node(
        package='platoon',
        executable='control_node',
        name='control_node',
        namespace=CAR_ID,
        output='screen',
        emulate_tty=True,
        parameters=[config_file]
    )

    # ESP32-S3 V2X 통신 노드
    v2x = Node(
        package='platoon',
        executable='v2x_node',
        name='v2x_node',
        namespace=CAR_ID,
        output='screen',
        emulate_tty=True,
        parameters=[{
            'vehicle_id': VEHICLE_ID,
            'is_designated_leader': IS_DESIGNATED_LEADER,
            'destination_id': DESTINATION_ID,
        }]
    )

    # 라이다 노드
    lidar = Node(
        package='platoon',
        executable='lidar_node',
        name='lidar_node',
        namespace=CAR_ID,
        output='screen',
        emulate_tty=True,
        parameters=[config_file]
    )

    return LaunchDescription([
        lane_detector,
        decision,
        control        
    ])