#!/bin/bash

set -e

current_path=$(cd $(dirname "${BASH_SOURCE[0]}") && pwd)
ROOTDIR=/
BUILDER_OUTDIR=/home/builder
CARPO_DEB=carpo_deb_repos
CARPO_DEBPATH=${ROOTDIR}${CARPO_DEB}
ROS2_GALACTIC_PATH=${ROOTDIR}opt/ros2/galactic
ROS2_CYBERDOG_PATH=${ROOTDIR}opt/ros2/cyberdog
ROS2_WS_PATH=${ROOTDIR}carpo_ws
DATETIME=$(date "+%Y-%m-%d_%H-%M-%S")

echo -n "clone cyberdog ros2 source code"
mkdir -p ${ROS2_WS_PATH}/src
cd ${ROS2_WS_PATH}/src
git clone git@git.n.xiaomi.com:MiRoboticsLab/rop/cyberdog_ws.git
vcs import . < cyberdog_ws/cyberdog.repos 
cd ..

echo -n "build cyberdog ros2 source code"
source ${ROS2_GALACTIC_PATH}/local_setup.bash
rm ${ROS2_CYBERDOG_PATH}/* -rf
colcon build --merge-install --install-base ${ROS2_CYBERDOG_PATH} --parallel-workers 40

echo -n "package cyberdog ros2 source code"
mkdir ${CARPO_DEBPATH}
cd ${CARPO_DEBPATH}
git clone git@git.n.xiaomi.com:MiRoboticsLab/deb_workspace/carpo_deb_ws/carpo_cyberdog_ros2_lib_deb.git
rm carpo_cyberdog_ros2_lib_deb/.git/ -rf
cp ${ROS2_CYBERDOG_PATH}/* ${CARPO_DEBPATH}/carpo_cyberdog_ros2_lib_deb/src${ROS2_CYBERDOG_PATH} -rf

tar -czf ${BUILDER_OUTDIR}/${CARPO_DEB}.tgz ${CARPO_DEBPATH}
echo finished!

