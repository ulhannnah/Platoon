import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # --- [차량 1호기 설정] ---
    CAR_ID = 'car1'
    VEHICLE_ID = 101
    IS_DESIGNATED_LEADER = True   # platoon1 = 리더
    DESTINATION_ID = 9999
    # UWB 실장치 연결 — 실측 확인 중, 문제 생기면 다시 True로
    ALLOW_UWB_LESS_JOIN = False
    # 카메라 정상이면 False 유지, 문제 생기면 True로
    ALLOW_CAMERA_LESS_JOIN = False
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

    # 플래툰 판단(FSM) 노드 — V2X 연동, 단독주행 decision_node 대신 이걸 씀
    fsm_decision = Node(
        package='platoon',
        executable='fsm_decision_node',
        name='fsm_decision_node',
        namespace=CAR_ID,
        output='screen',
        emulate_tty=True,
        parameters=[{
            'vehicle_id': VEHICLE_ID,
            'is_designated_leader': IS_DESIGNATED_LEADER,
            'destination_id': DESTINATION_ID,
            'allow_camera_less_join': ALLOW_CAMERA_LESS_JOIN,
            'allow_uwb_less_join': ALLOW_UWB_LESS_JOIN,
        }]
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