import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # --- [차량 1호기 설정] ---
    CAR_ID = 'car1'
    VEHICLE_ID = 101
    IS_DESIGNATED_LEADER = True   # platoon1 = 리더
    # 팔로워 전용 — 리더 차량은 0(미사용)으로 둔다.
    LEADER_ID = 0
    INITIAL_PARTNER_ID = 0
    # 리더 전용 — 뒤에 붙을 것으로 예정된 팔로워 ID들(콤마 구분). 팔로워는 빈 문자열.
    EXPECTED_FOLLOWER_IDS = '102,103'
    DEFAULT_JOIN_LANE = 1   # §3 — JOIN 시 목표 차선(lane1)
    DEFAULT_EXIT_LANE = 2   # EXIT 시 목표 차선(lane2)
    # 이 차량이 처음 출발하는 차선. Platoon1은 이미 lane1에서 출발.
    # Platoon2/3(진입로 대기)는 반드시 2로 설정해야 JOIN 차선변경이 트리거됨.
    INITIAL_LANE = 1
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
    # cruise_speed=1 — 리더는 계속 저속으로 주행 (config_file 다음에 와서 그 값을 덮어씀)
    decision = Node(
        package='platoon',
        executable='decision_node',
        name='decision_node',
        namespace=CAR_ID,
        output='screen',
        emulate_tty=True,
        parameters=[config_file, {'cruise_speed': 1}]
    )

    # 플래툰 판단(FSM) 노드 — fsm_test.py 기반. decision_node를 파라미터/서비스로
    # 조작만 하고 vehicle_cmd는 직접 발행하지 않는다 (조향 소유권은 decision_node 전담).
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
            'leader_id': LEADER_ID,
            'initial_partner_id': INITIAL_PARTNER_ID,
            'expected_follower_ids': EXPECTED_FOLLOWER_IDS,
            'default_join_lane': DEFAULT_JOIN_LANE,
            'default_exit_lane': DEFAULT_EXIT_LANE,
            'initial_lane': INITIAL_LANE,
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
        control,
        fsm_decision,
        v2x,
    ])