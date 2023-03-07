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

import getopt
import os
import platform
import re
import socket
import subprocess
import sys
import time
import uuid


#
# 预处理：仅处理命令行参数，详情参见 help_info
#
def preprocessing():
    argv = sys.argv[0:]
    sys.argv = sys.argv[0:3]
    if len(argv) > 4:
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
        print('[', user, '] [mi] Preprocessing(', str(argv[4:]), ')...')
        for now_arg in argv[4:]:
            if re.match(r'^(mi:=)+.*$', now_arg):
                arg_str = re.sub(r'^(mi:=)+', '', now_arg)
                arg_list = arg_str.split()
                opts, args = getopt.getopt(
                    arg_list, '-h-v-r-d:', ['help', 'version', 'reboot', 'data=']
                )
                for opt_name, opt_value in opts:
                    if opt_name in ('-h', '--help'):
                        print('[', user, '] [mi] Help info:', help_info)
                        exit()
                    if opt_name in ('-v', '--version'):
                        print('[', user, '] [mi] Version info:', version_info)
                        exit()
                    if opt_name in ('-r', '--reboot'):
                        target_ps = ' '.join(argv[0:4])
                        _ps = "ps -ef | grep '" + target_ps
                        _find = "' | wc -l"
                        _kill = "' | awk 'NR==1'| awk '{print $2}' | xargs kill -9"
                        find_cmd = _ps + _find
                        kill_cmd = _ps + _kill
                        "' | awk 'NR==1'| awk '{print $2}' | xargs kill -9"
                        while True:
                            find_cmd_output = os.popen(find_cmd, 'r')
                            size_int = int(
                                re.sub(r'[^0-9]+', '', find_cmd_output.read())
                            )
                            if size_int > 3:
                                kill_cmd_output = os.popen(kill_cmd, 'r')
                                print(
                                    '[',
                                    user,
                                    '] [mi] Stopping[',
                                    int(size_int - 4),
                                    ']: ',
                                    target_ps,
                                    kill_cmd_output,
                                )
                                time.sleep(2)
                            else:
                                print('[', user, '] [mi] About to start:', target_ps)
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
# 获取 shell 指令结果
#
def get_shell(cmd):
    cmd_ret = subprocess.Popen(
        cmd,
        shell=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        encoding='utf_8',
    )
    data = cmd_ret.communicate()[0]
    return data


#
# 获取 计算机 MAC 地址
# 默认网卡：随机第一个非0地址
# cat /sys/class/net/xxx/address
# ifconfig xxx | grep ether | awk 'NR==1' |awk '{print $2}'
#
def get_mac(target_network_card_name=''):
    mac = '00:00:00:00:00:00'
    net_list = get_shell('ls /sys/class/net/').split('\n')
    if len(target_network_card_name) == 0:
        for net in net_list:
            mac = ''
            if len(net) == 0:
                continue
            net = re.sub('[^0-9a-zA-Z:]+', '', net)
            mac = get_shell('cat /sys/class/net/' + net + '/address').strip()
            if mac == '00:00:00:00:00:00':
                continue
            if len(mac) != 0:
                break
    else:
        if net_list.count(target_network_card_name):
            mac = get_shell(
                'cat /sys/class/net/' + target_network_card_name + '/address'
            ).strip()
    if len(mac) == 0:
        mac = uuid.UUID(int(uuid.getnode())).hex[-12:]
        mac = ':'.join([mac[e: e + 2] for e in range(0, 11, 2)])
    mac = re.sub('[:]+', '_', mac)
    return mac


#
# 获取 namespace
#
def get_namespace():
    hostname = socket.getfqdn(socket.gethostname())
    mac = get_mac('eth0')
    namespace = hostname + '_' + mac
    namespace = re.sub('[^0-9a-zA-Z]+', '_', namespace)
    return namespace
