# Copyright 2016 Open Source Robotics Foundation, Inc.
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
import re
import sys
import getopt
import time
import socket
import platform
import uuid


#
# 预处理：仅处理命令行参数，详情参见 help_info
#
def preprocessing():
    if (len(sys.argv) > 4):
        version_info = """
version: 0.0.3 (default, 4 9 2022, 20:11:20)
Python: 3.8.10 (default, Mar 15 2022, 12:22:08)  [GCC 9.4.0]
"""

        help_info = """
usage: ros2 launch package_name [launch_file_name] mi:="[-h] [-v] [-r]" \
[launch_arguments [launch_arguments ...]]

Run a launch file

positional arguments:
  package_name          Name of the ROS package which contains the launch file
  launch_file_name      Name of the launch file
  launch_arguments      Arguments to the launch file; '<name>:=<value>' \
(for duplicates, last one wins)

optional arguments:
  -h, --help            Show this help message and exit.
  -v, --version         Show version information.
  -r, --reboot          Restart the current process.
                        If the current program is already started, close all historical processes \
of the current program before starting it.
                        Start the current program if it is not already started.
  -d, --data            Reserve option
"""

        user_env_var = 'USERNAME' if platform.system() == 'Windows' else 'USER'
        user = os.environ[user_env_var]
        print('[', user, '] [mi] Preprocessing(', str(sys.argv[4:]), ')...')
        for now_arg in sys.argv[4:]:
            if re.match(r'^(mi:=)+.*$', now_arg):
                arg_str = re.sub(r'^(mi:=)+', '', now_arg)
                arg_list = arg_str.split()
                opts, args = getopt.getopt(
                    arg_list, '-h-v-r-d:',
                    ['help', 'version', 'reboot', 'data='])
                for opt_name, opt_value in opts:
                    if opt_name in ('-h', '--help'):
                        print('[', user, '] [mi] Help info:', help_info)
                        exit()
                    if opt_name in ('-v', '--version'):
                        print('[', user, '] [mi] Version info:', version_info)
                        exit()
                    if opt_name in ('-r', '--reboot'):
                        target_ps = ' '.join(sys.argv[0:4])
                        _ps = "ps -ef | grep '" + target_ps
                        _find = "' | wc -l"
                        _kill = "' | awk 'NR==1'| awk '{print $2}' | xargs kill -9"
                        find_cmd = _ps + _find
                        kill_cmd = _ps + _kill
                        "' | awk 'NR==1'| awk '{print $2}' | xargs kill -9"
                        while True:
                            find_cmd_output = os.popen(find_cmd, 'r')
                            size_int = int(
                                re.sub(r'[^0-9]+', '', find_cmd_output.read()))
                            if size_int > 3:
                                kill_cmd_output = os.popen(kill_cmd, 'r')
                                print('[', user, '] [mi] Stopping[',
                                      int(size_int - 4), ']: ', target_ps,
                                      kill_cmd_output)
                                time.sleep(2)
                            else:
                                print('[', user, '] [mi] About to start:',
                                      target_ps)
                                break
                    if opt_name in ('-d', '--data'):
                        data_value = opt_value
                        print('[', user, '] [*] data is ', data_value)
                        # do something
                        exit()
            else:
                continue
        else:
            print('[', user, '] [mi] Preprocessing is complete')


#
# 获取 namespace：主机名 + MAC地址
#
def get_namespace():
    mac = uuid.UUID(int=uuid.getnode()).hex[-12:]
    mac_address = '_'.join([mac[e:e + 2] for e in range(0, 11, 2)])
    hostname = socket.getfqdn(socket.gethostname())
    namespace = hostname + '_' + mac_address
    return namespace
