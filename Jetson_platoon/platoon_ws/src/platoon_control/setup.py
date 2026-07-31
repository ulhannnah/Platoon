from setuptools import find_packages, setup

package_name = "platoon_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/platoon_control_launch.py"]),
    ],
    install_requires=["setuptools", "opencv-python", "numpy", "pyserial", "pynput"],
    zip_safe=True,
    maintainer="your_name",
    maintainer_email="you@example.com",
    description="플래툰(군집주행) 판단 및 제어 노드",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "platoon_control_node = platoon_control.platoon_control_node:main",
        ],
    },
)
