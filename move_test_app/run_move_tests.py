from __future__ import annotations

import argparse
import itertools
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


def axis_num_to_letter(axis_num: int) -> str:
    if axis_num < 1:
        raise ValueError(f"Axis number must be >= 1, got {axis_num}")
    return chr(ord("A") + axis_num - 1)


@dataclass
class PathPoint:
    target_positions_deg: List[float]
    target_velocity_part_of_1: float
    target_acceleration_part_of_1: float

    def __init__(
        self,
        target_positions_deg,
        target_velocity_part_of_1,
        target_acceleration_part_of_1,
    ) -> None:
        self.target_positions_deg = target_positions_deg
        self.target_velocity_part_of_1 = target_velocity_part_of_1
        self.target_acceleration_part_of_1 = target_acceleration_part_of_1
        self.build_map_command()

    def build_map_command(self):
        self.string_representation = "MAP "
        for i in range(0, len(self.target_positions_deg)):
            self.string_representation += (
                f"J{axis_num_to_letter(i + 1)}{self.target_positions_deg[i]} "
            )
        self.string_representation += f"SP{self.target_velocity_part_of_1}"
        self.string_representation += f"AC{self.target_acceleration_part_of_1}"

    def __str__(self) -> str:
        return self.string_representation


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


if __name__ == "__main__":
    sys.exit(main())
