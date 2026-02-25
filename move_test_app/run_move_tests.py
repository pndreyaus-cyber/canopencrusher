from __future__ import annotations

import argparse
import itertools
import re
import signal
import sys
import time
from dataclasses import dataclass, asdict
from datetime import datetime
from pathlib import Path
from typing import Iterable, List, Optional

import serial  # type: ignore
from openpyxl import Workbook

AXES_NUM = 5
LAST_CHAR = 'E'

def axis_num_to_letter(axis_num: int) -> str:
    if axis_num < 1:
        raise ValueError(f"Axis number must be >= 1, got {axis_num}")
    return chr(ord("A") + axis_num - 1)


@dataclass
class PathPoint:
    target_positions_deg: List[float]
    target_velocity_part_of_1: float
    target_acceleration_part_of_1: float


@dataclass
class PathPointResult:
    result_positions: List[float]
    result_move_time_ms: float
    reached_target: bool
    status: str
    error: str
    timestamp_utc: str


class SerialRobotClient:
    def __init__(self, port: str, baud: int, timeout_s: float = 0.15) -> None:
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=timeout_s)

    def close(self) -> None:
        if self.ser and self.ser.is_open:
            self.ser.close()

    def clear_input_buffer(self) -> None:
        self.ser.reset_input_buffer()

    def send_command(self, command: str) -> None:
        payload = f"{command}\n".encode("ascii", errors="ignore")
        self.ser.write(payload)
        print(f"Sending command: {command}")
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

    def wait_for_prefix(self, prefixes: Iterable[str], timeout_s: float) -> Optional[str]:
        end_t = time.perf_counter() + timeout_s
        prefixes_tuple = tuple(prefixes)
        while time.perf_counter() < end_t:
            line = self.read_line()
            if line is None:
                continue
            if line.startswith(prefixes_tuple):
                return line
        return None


def parse_rpp_position(line: str, axis_letter: str) -> Optional[float]:
    token_regex = rf"J{re.escape(axis_letter)}([+-]?\d+(?:\.\d+)?)"
    match = re.search(token_regex, line)
    if not match:
        return None
    return float(match.group(1))


def build_map_command(target_positions_deg: List[float], velocity_pct: float, acceleration_pct: float) -> str:
    command = "MAP "
    
    for node_id in range(ord('A'), ord(LAST_CHAR) + 1):
        command += ("J" + chr(node_id) + str(target_positions_deg[node_id - ord('A')]))

    command += ("SP" + str(velocity_pct) + " AC" + str(acceleration_pct))

    return command


def build_rpp_command(axis_num: int) -> str:
    axis_letter = axis_num_to_letter(axis_num)
    return f"RPPJ{axis_letter}"

def angle_to_steps(angle_deg: float) -> int:
    return int(angle_deg * 32768 * 50 / 360)


def run_next_PathPoint(
    client: SerialRobotClient,
    case: PathPoint,
        position_tolerance_deg: float,
    move_timeout_s: float,
    poll_period_s: float,
) -> PathPointResult:
    
    map_command = build_map_command(
        case.target_positions_deg,
        case.target_velocity_part_of_1,
        case.target_acceleration_part_of_1
    )

    rpp_command = "RPP"
    client.clear_input_buffer()
    start_t = time.perf_counter()
    client.send_command(map_command)
    print("Waiting for MAP response...")

    map_reply = client.wait_for_prefix(prefixes=["MAP", "MAJ"], timeout_s=4)
    print(f"MAP reply: {map_reply}")
    if map_reply and (" IP" in map_reply or " IC" in map_reply or " FF" in map_reply):
        return PathPointResult(
            result_positions=[],
            result_move_time_ms=0.0,
            reached_target=False,
            status="COMMAND_REJECTED",
            error=map_reply,
            timestamp_utc=datetime.now().isoformat(timespec="seconds"),
        )
    

    parsed_positions = []
    target_reached = False

    while (not target_reached) and ((time.perf_counter() - start_t) <= move_timeout_s):
        client.send_command(rpp_command)
        rpp_line = client.wait_for_prefix(prefixes=["RPP"], timeout_s=0.3)
        if not rpp_line:
            continue

        print("RPP reply:", rpp_line)
        rpp_line_split = rpp_line.split()
        if rpp_line_split[1] != "OK":
            print("RPP Fail")
            break

        success = True
        for node_id in range(ord('A'), ord(LAST_CHAR) + 1):
            parsed_position = parse_rpp_position(rpp_line, chr(node_id))
            if parsed_position is None:
                success = False
                break
            parsed_positions.append(parsed_position)

        if not success:
            print("RPP COMMAND FAILED")
            break
        
        rpp_status = ""
        for node_id in range(ord('A'), ord(LAST_CHAR) + 1):
            parsed_position = parsed_positions[node_id - ord('A')]
            target_position = case.target_positions_deg[node_id - ord('A')]
            if abs(target_position - parsed_position) <= position_tolerance_deg:
                target_reached = True
            else:
                target_reached = False
        
        if not target_reached:
            print("Target not yet reached")


    move_time_ms = (time.perf_counter() - start_t) * 1000.0

    return PathPointResult(parsed_positions,
                            move_time_ms,
                            target_reached,
                            "",
                            "",
                            timestamp_utc=datetime.now().isoformat(timespec="seconds"))

def generate_path_from_txt(file_path: Path) -> List[PathPoint]:
    print(f"Generating path from text {file_path}")
    path = []
    with open(file_path, "r") as file:
        line = file.readline()
        while line != "":
            params = line.split()
            print(f"Read line: {line}")
            target_positions = [float(params[i]) for i in range(0, AXES_NUM)]
            print("Target positions:")
            print(target_positions)
            target_velocity = float(params[AXES_NUM])
            target_acceleration = float(params[AXES_NUM + 1])
            print(f"Target velocity={target_velocity:0.3f}, target acceleration={target_acceleration:0.3f}")
            path.append(PathPoint(target_positions_deg=target_positions,
                                  target_velocity_part_of_1=target_velocity,
                                  target_acceleration_part_of_1=target_acceleration))
            line = file.readline()
    return path

def main() -> int:
    parser = argparse.ArgumentParser(description="Automated MAP move command tester for CANCrusher")
    parser.add_argument("--port", required=True, help="Serial port, e.g. COM7")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument("--axes", default="6", help="Axis numbers, comma-separated")
    parser.add_argument("--input_file", required=True, type=str)

    parser.add_argument("--position-tolerance", type=float, default=0.5, help="Accepted position error in degrees")
    parser.add_argument("--move-timeout", type=float, default=8.0, help="Timeout per test in seconds")
    parser.add_argument("--poll-ms", type=int, default=50, help="RPP polling period in ms")
    args = parser.parse_args()

    print("Arguments parsed")
    path = generate_path_from_txt(Path(args.input_file))

    total_points = len(path)
    print(f"Path consists of {total_points} points")
    aborted = False

    def handle_sigint(_sig, _frame):
        nonlocal aborted
        aborted = True
        print("\\nStop requested. Finishing current test and saving partial results...")

    signal.signal(signal.SIGINT, handle_sigint)

    client = SerialRobotClient(port=args.port, baud=args.baud)

    for point_num, point in enumerate(path):
        print(f"==== Point {point_num} ====")
        result = run_next_PathPoint(client=client,
                           case=point,
                           position_tolerance_deg=args.position_tolerance,
                           move_timeout_s = args.move_timeout,
                           poll_period_s=args.poll_ms / 1000.0,)
        
        print(f"=== Point {point_num}: {"Target reached" if result.reached_target else "Target not reached"}; movement time: {(result.result_move_time_ms/1000.0):0.4f}s ===\n")
        


    client.close()

    if aborted:
        return 130
    return 0


if __name__ == "__main__":
    sys.exit(main())
