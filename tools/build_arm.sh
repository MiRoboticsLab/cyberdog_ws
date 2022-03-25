#!/bin/bash

set -e

current_path=$(cd $(dirname "${BASH_SOURCE[0]}") && pwd)
ROS2_GALACTIC_PATH=/opt/ros2/galactic
ROS2_CYBERDOG_PATH=/opt/ros2/cyberdog
DATETIME=$(date "+%Y-%m-%d_%H-%M-%S")

echo -n "clone carpo_cyberdog_ros2_lib_deb directories"
git clone git@git.n.xiaomi.com:MiRoboticsLab/deb_workspace/carpo_deb_ws/carpo_cyberdog_ros2_lib_deb.git
rm carpo_cyberdog_ros2_lib_deb/.git -rf

source /opt/ros2/galactic/local_setup.bash

echo -n "clone cyberdog ros2 applications source code"
mkdir -p /carpo_ws/src
cd /carpo_ws/src
git clone git@git.n.xiaomi.com:MiRoboticsLab/rop/cyberdog_ws.git
vcs import . < cyberdog_ws/cyberdog.repos
cd ..

echo -n "build cyberdog ros2 applications source code"
colcon build --merge-install --install-base /opt/ros2/cyberdog --parallel-workers 40 --packages-select=protocol

echo -n "get commit id sum of all repos"
cd /
git clone git@git.n.xiaomi.com:MiRoboticsLab/os/toolbox/carpo_deb_repos.git -b t_dev
cd carpo_deb_repos/version_manager_tools
./get_commitid_sum.sh ros_platform_code.xml carpo-cyberdog-ros2-lib
cp ros_platform_code.commitidsum /

cd /
cp /opt/ros2/cyberdog/* /carpo_cyberdog_ros2_lib_deb/src/opt/ros2/cyberdog/ -rf

tar -czf /home/builder/carpo_deb_repos.tgz /carpo_cyberdog_ros2_lib_deb/ ros_platform_code.commitidsum
echo finished!
