import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # --- [차량 2호기 설정] ---
    CAR_ID = 'car2'
    VEHICLE_ID = 102
    IS_DESIGNATED_LEADER = False   # platoon2 = 팔로워
    LEADER_ID = 101                # 전체 리더(Platoon1) ID
    INITIAL_PARTNER_ID = 101       # JOIN 시 바로 붙을 내 앞차 = 리더 자신
    EXPECTED_FOLLOWER_IDS = ''     # 팔로워는 미사용
    DEFAULT_JOIN_LANE = 1          # §3 — JOIN 시 목표 차선(lane1)
    DEFAULT_EXIT_LANE = 2          # EXIT 시 목표 차선(lane2)
    # 진입로에서 대기하는 차량이라 리더(lane1)와 다른 차선에서 시작.
    # 이 값 틀리면 JOIN 목표(lane1)와 같아 보여 차선변경이 트리거 안 됨.
    INITIAL_LANE = 2
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

    # 단독주행 노드 (조향/차선변경 전담 — fsm_decision_node는 이걸 파라미터/서비스로 조작만 함)
    decision = Node(
        package='platoon',
        executable='decision_node',
        name='decision_node',
        namespace=CAR_ID,
        output='screen',
        emulate_tty=True,
        parameters=[config_file]
    )

    # 플래툰 판단(FSM) 노드 — fsm_test.py 기반. vehicle_cmd는 직접 발행하지 않음.
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

    return LaunchDescription([
        lane_detector,
        decision,
        control,
        fsm_decision,
        v2x,
    ])
