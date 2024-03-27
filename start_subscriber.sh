#!/usr/bin/bash
source "/opt/ros/$ROS_DISTRO/setup.bash"
source /app/install/setup.sh
cd /app/scripts
python edge_subscriber.py