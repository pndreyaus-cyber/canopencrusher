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

def axis_num_to_letter(axis_num: int) -> str:
    if axis_num < 1:
        raise ValueError(f"Axis number must be >= 1, got {axis_num}")
    return chr(ord("A") + axis_num - 1)

@dataclass
class PathPoint:
    target_positions_deg: List[float]
    target_velocity_part_of_1: float
    target_acceleration_part_of_1: float

    def __init__(self, target_positions_deg, target_velocity_part_of_1, target_acceleration_part_of_1) -> None:
        self.target_positions_deg = target_positions_deg
        self.target_velocity_part_of_1 = target_velocity_part_of_1
        self.target_acceleration_part_of_1 = target_acceleration_part_of_1
        self.build_map_command()

    def build_map_command(self):
        self.string_representation = "MAP "
        for i in range(0, len(self.target_positions_deg)):
            self.string_representation += f"J{axis_num_to_letter(i + 1)}{self.target_positions_deg[i]} " 
        self.string_representation += f"SP{self.target_velocity_part_of_1}"
        self.string_representation += f"AC{self.target_acceleration_part_of_1}"

    def __str__(self) -> str:
        return self.string_representation
    
@dataclass
class PathPointResult:
    result_positions: List[float]
    result_move_time_ms: float
    reached_target: bool
    status: str
    error: str
    timestamp_utc: str

def create_error_path_point_result(error: str):
    return PathPointResult(
            result_positions=[],
            result_move_time_ms=0.0,
            reached_target=False,
            status="ERROR",
            error=error,
            timestamp_utc=datetime.now().isoformat(timespec="seconds"),
        )

class SerialRobotClient:
    def __init__(self, port: str, baud: int, axes_num: int, timeout_s: float = 0.15) -> None:
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
            elif line.startswith("CPM"):
                print(f"MoveParams: {line}")
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

def build_rpp_command(axis_num: int) -> str:
    axis_letter = axis_num_to_letter(axis_num)
    return f"RPPJ{axis_letter}"

def angle_to_steps(angle_deg: float) -> int:
    return int(angle_deg * 32768 * 50 / 360)


def run_next_PathPoint(
    client: SerialRobotClient,
    point: PathPoint,
    position_tolerance_deg: float,
    move_timeout_s: float,
    poll_period_s: float,
) -> PathPointResult:

    rpp_command = "RPP"
    client.clear_input_buffer()
    start_t = time.perf_counter()
    client.send_command(str(point))
    print("Waiting for MAP response...")

    map_reply = client.wait_for_prefix(prefixes=["MAP", "MAJ"], timeout_s=move_timeout_s)
    print(f"MAP reply: {map_reply}")
    if map_reply and (" IP" in map_reply or " IC" in map_reply or " FF" in map_reply):
        return create_error_path_point_result("MAP NOT SUCCESS")

    parsed_positions = []
    target_reached = False

    move_time_ms = (time.perf_counter() - start_t) * 1000.0

    client.send_command(rpp_command)
    rpp_line = client.wait_for_prefix(prefixes=["RPP"], timeout_s=poll_period_s)
    if not rpp_line:
        print("RPP request timeout! Aboring")
        return create_error_path_point_result("RPP RESPONSE TIMEOUT")
    
    rpp_line_split = rpp_line.split()
    if rpp_line_split[1] != "OK":
        return create_error_path_point_result("RPP FAIL")

    success = True
    for node_id in range(1, client.get_axes_num() + 1):
        parsed_position = parse_rpp_position(rpp_line, axis_num_to_letter(node_id))
        if parsed_position is None:
            success = False
            return create_error_path_point_result("POSITION NOT PARSED PROPERLY")
        parsed_positions.append(parsed_position)

    rpp_status = ""
    target_reached = True
    for node_id in range(0, client.get_axes_num()):
        parsed_position = parsed_positions[node_id]
        target_position = point.target_positions_deg[node_id]
        if abs(target_position - parsed_position) <= position_tolerance_deg:
            target_reached &= True
        else:
            print(f"Axis {node_id + 1}: parsed_position={parsed_position}; target_position={target_position}")
            target_reached &= False
        
    if not target_reached:
        print("Target not yet reached")

    return PathPointResult(parsed_positions,
                            move_time_ms,
                            target_reached,
                            "OK",
                            "",
                            timestamp_utc=datetime.now().isoformat(timespec="seconds"))

def generate_path_from_txt(file_path: Path, axes_num: int) -> List[PathPoint]:
    print(f"Generating path from file: {file_path}")
    path = []
    with open(file_path, "r") as file:
        line = file.readline()
        while line != "":
            params = line.split()
            target_positions = [float(params[i]) for i in range(0, axes_num)]
            target_velocity = float(params[axes_num])
            target_acceleration = float(params[axes_num + 1])

            path.append(PathPoint(target_positions_deg=target_positions,
                                  target_velocity_part_of_1=target_velocity,
                                  target_acceleration_part_of_1=target_acceleration))
            print(path[-1])

            line = file.readline()
    return path

def main() -> int:
    parser = argparse.ArgumentParser(description="Automated MAP move command executor for CANCrusher")
    parser.add_argument("--port", required=True, help="Serial port, e.g. COM7")
    parser.add_argument("--input-file", required=True, type=str)
    parser.add_argument("--axes", required=True, type=int, help="Number of axes to use")

    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument("--position-tolerance", type=float, default=10, help="Accepted position error in steps")
    parser.add_argument("--move-timeout", type=float, default=10, help="Timeout per move in seconds")
    parser.add_argument("--poll-ms", type=int, default=500, help="RPP polling period in ms")
    args = parser.parse_args()

    path = generate_path_from_txt(Path(args.input_file), args.axes)

    total_points = len(path)
    print(f"Path consists of {total_points} points")
    aborted = False

    def handle_sigint(_sig, _frame):
        nonlocal aborted
        aborted = True
        print("\\nStop requested. Finishing current test and saving partial results...")

    signal.signal(signal.SIGINT, handle_sigint)

    client = SerialRobotClient(port=args.port, baud=args.baud, axes_num=args.axes)

    for point_num, point in enumerate(path):
        print(f"==== Point {point_num} ====")
        result = run_next_PathPoint(client=client,
                                    point=point,
                                    position_tolerance_deg=args.position_tolerance,
                                    move_timeout_s=args.move_timeout,
                                    poll_period_s=args.poll_ms / 1000.0)
        
        status = "Target reached" if result.reached_target else "Target not reached"
        print(f"=== Point {point_num}: {status}; movement time: {(result.result_move_time_ms/1000.0):0.4f}s ===\n")
        
        if not result.reached_target:
            print("Aborting path. Target was not reached")
            #break

        if aborted:
            client.close()
            return 130

    client.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
