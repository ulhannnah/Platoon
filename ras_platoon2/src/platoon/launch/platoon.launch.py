import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # --- [차량 2호기 설정] ---
    CAR_ID = 'car2'
    VEHICLE_ID = 102
    IS_DESIGNATED_LEADER = False   # platoon2 = 팔로워
    DESTINATION_ID = 9999
    # UWB 실장치 연결 — 실측 확인 중, 문제 생기면 다시 True로
    ALLOW_UWB_LESS_JOIN = False
    # 카메라 미연결 — 차선정렬 조건 생략하고 테스트
    ALLOW_CAMERA_LESS_JOIN = True
    # -------------------------

    config_file = os.path.expanduser('~/ros2_ws/src/platoon/config/platoon_params.yaml')

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

    # STM32 제어 / UART 통신 노드 (지금 STM 미연결 — 연결하면 아래 return에서 주석 해제)
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
        # lane_detector,  # 카메라 미연결 — 연결하면 주석 해제
        fsm_decision,
        # control,  # STM 미연결 — 연결하면 주석 해제
        v2x
    ])