#!/usr/bin/python3
#
# Copyright (c) 2021 Xiaomi Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import launch

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers.on_shutdown import OnShutdown
from athena_pycommon import Get_Namespace
from launch_ros.actions import Node


def generate_launch_description():

    # Define arguments
    Get_Namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value=Get_Namespace(),
        description='Namespace of all nodes')
    shutdown_cmd = RegisterEventHandler(
        OnShutdown(on_shutdown=[LogInfo(msg="ROS Apps is exiting.")]))

    namespace = LaunchConfiguration('namespace')

    # --- slam and nav ---
    # Declare the launch arguments
    realsense2_camera_dir = get_package_share_directory('realsense2_camera')
    realsense2_camera_launch_dir = os.path.join(realsense2_camera_dir,
                                                'launch')
    rtabmap_ros_dir = get_package_share_directory('rtabmap_ros')
    rtabmap_ros_launch_dir = os.path.join(rtabmap_ros_dir, 'launch')
    object_tracking_dir = get_package_share_directory('athena_tracking')
    object_tracking_launch_dir = os.path.join(object_tracking_dir, 'launch')
    movebase_dir = get_package_share_directory('move_base2')
    movebase_launch_dir = os.path.join(movebase_dir, 'launch')
    open_vins_dir = get_package_share_directory('ov_msckf')
    open_vins_launch_dir = os.path.join(open_vins_dir, 'launch')

    realsense_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense2_camera_launch_dir, 'on_dog.py')),
        launch_arguments={'namespace': namespace}.items())
    rtabmap_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rtabmap_ros_launch_dir, 'rs_d455.launch.py')),
        launch_arguments={'namespace': namespace}.items())
    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(movebase_launch_dir, 'tracking_launch_mb.py')),
        launch_arguments={'namespace': namespace}.items())
    object_tracking_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(object_tracking_launch_dir, 'launch.py')),
        launch_arguments={'namespace': namespace}.items())
    open_vins_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(open_vins_launch_dir, 'ros2.launch.py')),
        launch_arguments={'namespace': namespace}.items())

    interactive_cmd = Node(package='interactive',
                           executable='interactive_node',
                           namespace=namespace,
                           name='interactive')
    # -------------------------

    # Create the launch description and populate
    ld = launch.LaunchDescription()

    # Add actions to launch
    ld.add_action(Get_Namespace_cmd)
    ld.add_action(shutdown_cmd)

    ld.add_action(interactive_cmd)
    ld.add_action(realsense_cmd)
    ld.add_action(rtabmap_cmd)
    ld.add_action(navigation_cmd)
    ld.add_action(object_tracking_cmd)
    ld.add_action(open_vins_cmd)
    return ld
