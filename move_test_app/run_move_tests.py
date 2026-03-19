from __future__ import annotations

import argparse
from html import parser
import itertools
import math
import re
import signal
import string
import sys
import time
from dataclasses import dataclass, asdict
from datetime import datetime
from pathlib import Path
from typing import Iterable, List, Optional, Tuple

import serial  # type: ignore

from ik1 import IkParameters, calc_ik_simple, calculate_angles
from ik2 import solve_robot

class SerialRobotClient:
    def __init__(
        self, port: str, baud: int, axes_num: int, timeout_s: float = 0.15
    ) -> None:
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=timeout_s)
        self.axes_num = axes_num

    def close(self) -> None:
        if self.ser and self.ser.is_open:
            self.ser.close()

    def clear_input_buffer(self) -> None:
        self.ser.reset_input_buffer()

    def send_command(self, command: str) -> None:
        payload = f"{command}\n".encode("ascii", errors="ignore")
        self.ser.write(payload)
        self.ser.flush()

    def read_line(self) -> Optional[str]:
        raw = self.ser.readline()
        if not raw:
            return None
        try:
            line = raw.decode("utf-8", errors="ignore").strip()
        except UnicodeDecodeError:
            return None
        return line if line else None

    def wait_for_prefix(self, prefix_str: str, timeout_s: float) -> Optional[str]:
        end_t = time.perf_counter() + timeout_s
        while time.perf_counter() < end_t:
            line = self.read_line()
            if line is None:
                continue
            if line.startswith(prefix_str):
                return line
            else:
                print(f"Received unrelated line: {line}")
        return None

    def get_axes_num(self) -> int:
        return self.axes_num


def parse_rpp_position(line: str, axis_letter: str) -> Optional[float]:
    token_regex = rf"J{re.escape(axis_letter)}([+-]?\d+(?:\.\d+)?)"
    match = re.search(token_regex, line)
    if not match:
        return None

    print(f"Match group: {match}")

    return float(match.group(1))


def generate_path_from_txt(
    file_path: Path, axes_num: int, contains_replies: bool
) -> List[Tuple[str, str]]:
    print(f"Generating path from file: {file_path}")
    commands = []
    with open(file_path, "r") as file:
        line = file.readline()
        while line != "":
            commands.append(
                (line.strip(), line.split()[0])
            )  # store command and its type (e.g. MAP, MAJ, etc.)
            if contains_replies:
                answer = file.readline().strip()  # skip reply

            line = file.readline()
    return commands


def run_commands(
    commands: List[Tuple[str, str]], client: SerialRobotClient, move_timeout_s: float
):
    for command in commands:
        print(f"Running command: {command[0]}")
        result = client.send_command(command[0])
        reply = client.wait_for_prefix(
            command[1], timeout_s=move_timeout_s
        )  # wait for the reply corresponding to the command type
        print(f"Result: {'Reply timeout' if reply is None else reply}")
        #time.sleep(1)  # small delay between commands


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Automated MAP move command executor for CANCrusher"
    )
    parser.add_argument("--port", required=True, help="Serial port, e.g. COM7")
    parser.add_argument("--input-file", required=True, type=str)
    parser.add_argument("--axes", required=True, type=int, help="Number of axes to use")

    parser.add_argument("-c", "--check-replies", action="store_true")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument(
        "--position-tolerance",
        type=float,
        default=10,
        help="Accepted position error in steps",
    )
    parser.add_argument(
        "--move-timeout", type=float, default=10, help="Timeout per move in seconds"
    )
    args = parser.parse_args()

    commands = generate_path_from_txt(Path(args.input_file), args.axes, args.check_replies)

    total_commands = len(commands)
    print(f"Path consists of {total_commands} commands")
    aborted = False

    def handle_sigint(_sig, _frame):
        nonlocal aborted
        aborted = True
        print("\\nStop requested. Finishing current command and saving partial results...")

    signal.signal(signal.SIGINT, handle_sigint)

    client = SerialRobotClient(port=args.port, baud=args.baud, axes_num=args.axes)

    run_commands(commands, client, move_timeout_s=args.move_timeout)

    client.close()
    return 0

def test_ik_1(port, baud, axes):
    client = SerialRobotClient(port=port, baud=baud, axes_num=axes)

    ik_params = IkParameters()
    result, joint_positions = calc_ik_simple(0.32, 0.35, 0.17, ik_params) # in meters
    print(result)
    print(joint_positions)
    print(list(map(lambda x: x * 180 / math.pi, joint_positions)))  # Convert radians to degrees
    if result:
        command = "MAP JA{:.2f} JB{:.2f} JC{:.2f} JD0.0 JE{:.2f} SP0.1 AC0.02".format(
            180 * joint_positions[0] / math.pi,
            -180 * joint_positions[1] / math.pi,
            180 * joint_positions[2] / math.pi,
            min(180 * joint_positions[3] / math.pi, 110),
        )
        print(f"Generated command: {command}")
        #run_commands([(command, "MAP")], client, move_timeout_s=20)
    #run_commands([("MAP " + " ".join(f"{chr(ord('A') + i)}{joint_positions[i]:.2f}" for i in range(len(joint_positions))), "MAP")], client, move_timeout_s=5)

def test_ik_2(port, baud, axes):
    client = SerialRobotClient(port=port, baud=baud, axes_num=axes)
    positions = [(-0.064, 0.395, 0.575)]
    for pos in positions:
        angles = calculate_angles(*pos)
        command = "MAP JA{:.2f} JB{:.2f} JC{:.2f} JD0.0 JE{:.2f} SP0.1 AC0.005".format(
                180 * angles[0] / math.pi,
                180 * angles[1] / math.pi,
                180 * angles[2] / math.pi,
                min(-180 * angles[3] / math.pi, 110),
            )
        print(f"Generated command: {command}")
        run_commands([(command, "MAP")], client, move_timeout_s=20)
        time.sleep(2)

def test_ik_3(port, baud, axes):
    client = SerialRobotClient(port=port, baud=baud, axes_num=axes)
    angles = solve_robot(0.0, 0.35, 0.4)
    if angles is None:
        print("IK solver failed to find a solution")
        return
    print("IK solver found angles (radians):", angles)
    command = "MAP JA{:.2f} JB{:.2f} JC{:.2f} JD0.0 JE{:.2f} SP0.1 AC0.02".format(
            180 * angles[0] / math.pi,
            180 * angles[1] / math.pi,
            180 * angles[2] / math.pi,
            min(-180 * angles[3] / math.pi, 110),
        )
    print(f"Generated command: {command}")
    #run_commands([(command, "MAP")], client, move_timeout_s=20)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Automated MAP move command executor for CANCrusher"
    )

    parser.add_argument("--port", required=True, help="Serial port, e.g. COM7")
    parser.add_argument("--axes", required=True, type=int, help="Number of axes to use")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    args = parser.parse_args()
    test_ik_2(args.port, args.baud, args.axes)
    #test_ik_3(args.port, args.baud, args.axes)
