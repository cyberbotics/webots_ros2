import os
import signal
import socket
import subprocess
import sys
from os import walk

HOST = ""  # Any host can connect
PORT = int(sys.argv[1])  # Port to listen on

shared_folder = None
webots = None

def start_webots(connection):
    #shared_folder_str = shared_folder.decode("utf-8")
    filenames = next(walk(shared_folder), (None, None, []))[2]
    worlds = list(filter(lambda file: file.endswith('.wbt'), filenames))

    if(len(worlds) == 0):
        return -1
    if(len(worlds) > 1):
        return -2
    world_file = os.path.join(shared_folder, worlds[0])

    arguments = []
    lines = []
    if 'launch_args.txt' in filenames:
        launch_args_file = open(os.path.join(shared_folder, 'launch_args.txt'), 'r')
        lines = launch_args_file.readlines()
        for i in range(len(lines)):
            lines[i] = lines[i].strip()

    connection.sendall(b"ACK")
    subprocess.call(['/usr/bin/open', '-W', '-n', '-a', '/Applications/Webots.app', '--args', world_file, *lines])

    os.remove(world_file)
    if 'launch_args.txt' in filenames:
        os.remove(os.path.join(shared_folder, 'launch_args.txt'))

    return 1

def keyboardInterruptHandler(signal, frame):
    if(shared_folder):
        if(os.path.isdir(shared_folder)):
            for filename in os.listdir(shared_folder):
                filepath = os.path.join(shared_folder, filename)
                if os.path.isfile(filepath):
                    os.remove(filepath)
    exit(0)

signal.signal(signal.SIGINT, keyboardInterruptHandler)
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    conn, addr = s.accept()
    with conn:
        print(f"Connected by {addr}")
        while True:
            data = conn.recv(1024)
            if not data:
                break
            shared_folder = data.decode("utf-8")
            success = start_webots(conn) if(os.path.isdir(shared_folder)) else 0
            if success == 0:
                conn.sendall(b"FAIL0")
                print(f"The shared folder '{data_str}' doesn't exist.")
            elif success == -1:
                conn.sendall(b"FAIL1")
                print(f"No world could be found in the shared folder.")
            elif success == -2:
                conn.sendall(b"FAIL2")
                print(f"More than one world was found in the shared folder.")
            elif success == 1:
                print(f"Webots was executed successfully.")
