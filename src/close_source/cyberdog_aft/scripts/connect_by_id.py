#!/usr/bin/env python3
# Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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

import sys

from protocol.srv import ConnectPc
import rclpy
from rclpy.node import Node


class ClientAsync(Node):
    def __init__(self):
        super().__init__('connect_pc')
        self.cli = self.create_client(ConnectPc, 'connect_pc')
        while not self.cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ConnectPc.Request()

    def send_request(self, pc_id):
        self.req.connect_which = pc_id
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()
    connect_pc_client = ClientAsync()
    response = connect_pc_client.send_request(int(sys.argv[1]))
    connect_pc_client.get_logger().info(
        'Result of connect_id:%d' %
        int(response.success))

    connect_pc_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
