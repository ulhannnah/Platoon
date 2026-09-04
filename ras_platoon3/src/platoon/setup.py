from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'platoon'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/platoon']),
        ('share/platoon', ['package.xml']),
        (os.path.join('share', 'platoon', 'launch'),
            glob('launch/*.launch.py')),
        # 아래 한 줄을 추가해 주십시오.
        (os.path.join('share', 'platoon', 'config'), 
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lane',
    maintainer_email='lane@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lane_detector_node = platoon.lane_detector_node:main',

            # 플래툰 주행할때는 이 코드로 (v2x_node도 같이 켜야 함 — nearby_vehicles 공급원)
            'fsm_decision_node = platoon.fsm_decision_node:main',
            'v2x_node = platoon.v2x_node:main',

            # 그냥 단독주행 할 때는 이 코드로
            'decision_node = platoon.decision_node:main',

            # STM32 제어 / UART 통신 노드
            'control_node = platoon.control_node:main',

            # 레이다
            'lidar_node = platoon.lidar_node:main',
        ],
    },
)
