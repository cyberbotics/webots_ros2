#!/usr/bin/env python3

# Copyright 1996-2023 Cyberbotics Ltd.
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

"""TCP client to start Webots on the host."""

import os
import socket
import subprocess
import sys
import time


def get_host_ip():
    try:
        output = subprocess.run(['ip', 'route'], check=True, stdout=subprocess.PIPE, universal_newlines=True)
        for line in output.stdout.split('\n'):
            fields = line.split()
            if fields and fields[0] == 'default':
                return fields[2]
        sys.exit('Unable to get host IP address.')
    except subprocess.CalledProcessError:
        sys.exit('Unable to get host IP address. \'ip route\' could not be executed.')


HOST = get_host_ip()  # Connect to host of the VM
PORT = 2000  # Port to connect to

tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
launch_arguments = ''
for arg in sys.argv:
    if arg.endswith('.py') or arg == '':
        continue
    elif arg.endswith('.wbt'):
        world_name = arg
    else:
        launch_arguments = launch_arguments + ' ' + arg


def host_shared_folder():
    shared_folder_list = os.environ['WEBOTS_SHARED_FOLDER'].split(':')
    return shared_folder_list[0]


while tcp_socket.connect_ex((HOST, PORT)) != 0:
    print('WARNING: Unable to start Webots. Please start the local simulation server on your host machine. Next connection '
          'attempt in 1 second.', file=sys.stderr)
    time.sleep(1)
command = 'webots ' + launch_arguments + ' ' + os.path.join(host_shared_folder(), world_name)
tcp_socket.sendall(command.encode('utf-8'))
data = tcp_socket.recv(1024)
message = data.decode('utf-8')
if message.startswith('FAIL'):
    sys.exit(message)
elif message == 'ACK':
    pass
else:
    sys.exit('Unknown message.')

# Check connection for closing message
data = tcp_socket.recv(1024)
message = data.decode('utf-8')
if not data:
    tcp_socket.close()
    sys.exit('Server interrupted connection.')
elif message == 'CLOSED':
    tcp_socket.close()
    sys.exit('Webots process was ended.')
else:
    print(f'Server sent: {message}')
