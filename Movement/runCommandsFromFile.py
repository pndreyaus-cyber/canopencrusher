from SerialRobotClient import SerialRobotClient
from typing import List, Tuple
import argparse

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from parser import create_parser


def run_commands_from_file(file_path: str, port: str, baudrate: int, axes_num: int, timeout: float = 0):
    if timeout == 0:
        client = SerialRobotClient(port, baudrate, axes_num)
    else:
        client = SerialRobotClient(port, baudrate, axes_num, timeout)
    cnt = 0

    try:
        with open(file_path, 'r') as file:
            commands = file.readlines()
        
        for command in commands:
            command = command.strip()
            if not command:
                continue
        
            command_type = command.split()[0]  # Get the command type (e.g., "MOVE", "STOP")
            print(f"Sending command: {command}")
            client.send_command(command)
            reply = client.wait_for_prefix(command_type)
            ok = True
            if reply is None:
                print(f"{command_type} reply timeout!")
                ok = False
            else:
                status = reply.split()[1]
                if status != "OK":
                    print(f"{command_type} failed: {reply}\nStopping commands....")
                    ok = False
                else:
                    print(reply)
                    cnt += 1
        
            if ok:
                break
        print(f"Successful lines: {cnt}")
    except Exception as e:
        print(f"An error occurred: {e}. Successful lines: {cnt}")

if __name__ == "__main__":
    parser = create_parser("Run commands from a file on the robot")
    
    parser.add_argument("file", type=str, help="Path to the file containing the commands")
    args = parser.parse_args()

    if args.timeout is not None:
        run_commands_from_file(args.file, args.port, args.baud, args.axes, args.timeout)
    else:
        run_commands_from_file(args.file, args.port, args.baud, args.axes)