from __future__ import annotations
import argparse
import math
import re
import signal
import time
from pathlib import Path
from typing import List, Optional, Tuple

from serialRobotClient import SerialRobotClient, run_commands


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
    print(f"Generating command list from file: {file_path}")
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

    commands = generate_path_from_txt(
        Path(args.input_file), args.axes, args.check_replies
    )

    total_commands = len(commands)
    print(f"Path consists of {total_commands} commands")
    aborted = False

    def handle_sigint(_sig, _frame):
        nonlocal aborted
        aborted = True
        print(
            "\\nStop requested. Finishing current command and saving partial results..."
        )

    signal.signal(signal.SIGINT, handle_sigint)

    client = SerialRobotClient(port=args.port, baud=args.baud, axes_num=args.axes)

    run_commands(commands, client, move_timeout_s=args.move_timeout)

    client.close()
    return 0