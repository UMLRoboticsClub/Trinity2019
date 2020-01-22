#!/bin/bash
set -e

# setup ros environment -- add to bashrc so we can use interactive shell
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
echo "source /home/ros/catkin_ws/devel/setup.bash" >> ~/.bashrc
source /opt/ros/$ROS_DISTRO/setup.bash
[ -e /home/ros/catkin_ws/devel/setup.bash ] && source /home/ros/catkin_ws/devel/setup.bash
exec "$@"
