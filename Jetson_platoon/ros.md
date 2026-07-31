cd ~/platoon_ws
colcon build --packages-select platoon_control
source install/setup.bash
sudo ros2 run platoon_control platoon_control_node --ros-args -p mode:=manual