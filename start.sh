source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash

ros2 launch quadruped_bringup real.launch.py 