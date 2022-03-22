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
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

import yaml


def read_yaml(path):
    with open(path, 'r') as f:
        yaml_data = yaml.load(f.read())
    return yaml_data


def parse_yaml() -> LaunchDescription:
    launch_directory = get_package_share_directory('fire')
    yaml_data = read_yaml(os.path.join(launch_directory, 'params', 'node.yaml'))
    launch_description_impl = LaunchDescription()
    for data in yaml_data['launch_nodes']:
        # print(data.keys(), data.values())
        define_name = list(data.keys())[0]
        launch_description_impl.add_action(
            Node(
                package=data[define_name]['package'],
                namespace='/mi/',
                executable=data[define_name]['executable'],
                output='screen',
                respawn=data[define_name]['respawn'],
            )
        )
    return launch_description_impl


def generate_launch_description():
    return parse_yaml()
