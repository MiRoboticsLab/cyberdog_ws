# Copyright (c) 2021 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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
# sensors/sensor_manager/launch/sensor_manager.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bool_parameters_arg = DeclareLaunchArgument(
        'bool_parameters', default_value='False'
    )

    sensor_manager_node = Node(
            namespace='cyberdog',
            package='cyberdog_bringup_test',
            executable='cyberdog_bringup_test',
            name='cyberdog_bringup_test_',
            output='screen',
            parameters=[{
                'bool_parameters': LaunchConfiguration('bool_parameters'),
            }]
        )

    return LaunchDescription([
        bool_parameters_arg,
        sensor_manager_node,
    ])
