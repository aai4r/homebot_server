#!/usr/bin/bash
source "/opt/ros/$ROS_DISTRO/setup.bash"
colcon build

cd "/opt/ros/$ROS_DISTRO/src"
colcon build --packages-select aai4r_edge_interfaces
. install/setup.bash
