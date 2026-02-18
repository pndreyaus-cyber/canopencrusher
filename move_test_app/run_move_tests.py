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


DEFAULT_POSITIONS_DEG = [5, 10, 20, 45, 70]
DEFAULT_ACCELERATION_PERCENT_VALUES = [1, 4, 6, 8, 10, 12]
DEFAULT_VELOCITY_PERCENT_VALUES = [5, 10, 20, 30, 40, 50]

def axis_num_to_letter(axis_num: int) -> str:
    if axis_num < 1:
        raise ValueError(f"Axis number must be >= 1, got {axis_num}")
    return chr(ord("A") + axis_num - 1)


@dataclass
class TestCase:
    axis: int
    target_position_deg: float
    target_velocity_percent: float
    target_acceleration_percent: float


@dataclass
class TestResult:
    axis: int
    target_position: float
    target_velocity: float
    target_acceleration: float
    result_position: Optional[float]
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
    if not line.startswith("RPP OK"):
        return None
    match = re.search(token_regex, line)
    if not match:
        return None
    return float(match.group(1))


def build_map_command(axis_num: int, target_deg: float, velocity_pct: float, acceleration_pct: float) -> str:
    axis_letter = axis_num_to_letter(axis_num)
    return f"MAPJ{axis_letter}{target_deg:+g}SP{velocity_pct:g}AC{acceleration_pct:g}"


def build_rpp_command(axis_num: int) -> str:
    axis_letter = axis_num_to_letter(axis_num)
    return f"RPPJ{axis_letter}"

def angle_to_steps(angle_deg: float) -> int:
    return int(angle_deg * 32768 * 50 / 360)


def run_single_test(
    client: SerialRobotClient,
    case: TestCase,
        position_tolerance_deg: float,
    move_timeout_s: float,
    poll_period_s: float,
    settle_delay_s: float,
) -> TestResult:
    axis_letter = axis_num_to_letter(case.axis)
    map_command = build_map_command(
        axis_num=case.axis,
        target_deg=case.target_position_deg,
        velocity_pct=case.target_velocity_percent,
        acceleration_pct=case.target_acceleration_percent,
    )
    rpp_command = build_rpp_command(case.axis)
    print("MAP command:", map_command)
    client.clear_input_buffer()
    client.send_command(map_command)
    print("Waiting for MAP response...")

    map_reply = client.wait_for_prefix(prefixes=["MAP", "MAJ"], timeout_s=0.5)
    if map_reply and (" IP" in map_reply or " IC" in map_reply or " FF" in map_reply):
        return TestResult(
            axis=case.axis,
            target_position=case.target_position_deg,
            target_velocity=case.target_velocity_percent,
            target_acceleration=case.target_acceleration_percent,
            result_position=None,
            result_move_time_ms=0.0,
            reached_target=False,
            status="COMMAND_REJECTED",
            error=map_reply,
            timestamp_utc=datetime.now().isoformat(timespec="seconds"),
        )

    start_t = time.perf_counter()
    last_position: Optional[float] = None
    reached_target = False
    target_position_steps = angle_to_steps(case.target_position_deg)

    while (time.perf_counter() - start_t) <= move_timeout_s:
        client.send_command(rpp_command)
        rpp_line = client.wait_for_prefix(prefixes=["RPP"], timeout_s=0.3)
        #print("RPP reply:", rpp_line)
        if rpp_line:
            parsed_position = parse_rpp_position(rpp_line, axis_letter)
            if parsed_position is not None:
                last_position = parsed_position
                if abs(parsed_position - target_position_steps) <= position_tolerance_deg:
                    reached_target = True
                    break
        #print("Sleeping for {:.1f} ms before next poll...".format(poll_period_s * 1000))
        time.sleep(poll_period_s)

    move_time_ms = (time.perf_counter() - start_t) * 1000.0
    status = "OK" if reached_target else "TIMEOUT"
    error = "" if reached_target else "Target not reached within timeout"
    print(f"Move completed in {move_time_ms:.1f} ms, reached_target={reached_target}, last_position={last_position}")
    time.sleep(settle_delay_s)

    return TestResult(
        axis=case.axis,
        target_position=case.target_position_deg,
        target_velocity=case.target_velocity_percent,
        target_acceleration=case.target_acceleration_percent,
        result_position=last_position,
        result_move_time_ms=move_time_ms,
        reached_target=reached_target,
        status=status,
        error=error,
        timestamp_utc=datetime.now().isoformat(timespec="seconds"),
    )


def generate_test_cases(
    axes: List[int],
    positions_deg: List[float],
    velocity_percent_values: List[float],
    acceleration_percent_values: List[float],
) -> List[TestCase]:
    tests: List[TestCase] = []
    for axis in axes:
        for position, velocity, acceleration in itertools.product(
            positions_deg,
            velocity_percent_values,
            acceleration_percent_values,
        ):
            tests.append(
                TestCase(
                    axis=axis,
                    target_position_deg=position,
                    target_velocity_percent=velocity,
                    target_acceleration_percent=acceleration,
                )
            )

            tests.append(
                TestCase(
                    axis=axis,
                    target_position_deg=0,
                    target_velocity_percent=velocity,
                    target_acceleration_percent=acceleration,
                )
            )
    return tests


def write_results_xlsx(output_path: Path, results: List[TestResult]) -> None:
    wb = Workbook()
    ws = wb.active
    ws.title = "MAP_RPP_Test_Results"

    headers = [
        "axis",
        "target_position",
        "target_velocity",
        "target_acceleration",
        "result_position",
        "result_move_time_ms",
        "reached_target",
        "status",
        "error",
        "timestamp_utc",
    ]
    ws.append(headers)

    for result in results:
        data = asdict(result)
        ws.append([
            data["axis"],
            data["target_position"],
            data["target_velocity"],
            data["target_acceleration"],
            data["result_position"],
            data["result_move_time_ms"],
            data["reached_target"],
            data["status"],
            data["error"],
            data["timestamp_utc"],
        ])

    output_path.parent.mkdir(parents=True, exist_ok=True)
    wb.save(output_path)


def parse_csv_numbers(text: str, cast_fn):
    return [cast_fn(x.strip()) for x in text.split(",") if x.strip()]


def main() -> int:
    parser = argparse.ArgumentParser(description="Automated MAP move command tester for CANCrusher")
    parser.add_argument("--port", required=True, help="Serial port, e.g. COM7")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument("--axes", default="4,5", help="Axis numbers, comma-separated")
    parser.add_argument(
        "--positions",
        default=",".join(str(v) for v in DEFAULT_POSITIONS_DEG),
        help="Target positions in degrees, comma-separated",
    )
    parser.add_argument(
        "--velocity-percent",
        default=",".join(str(v) for v in DEFAULT_VELOCITY_PERCENT_VALUES),
        help="Velocity percentages, comma-separated",
    )
    parser.add_argument(
        "--acceleration-percent",
        default=",".join(str(v) for v in DEFAULT_ACCELERATION_PERCENT_VALUES),
        help="Acceleration percentages, comma-separated",
    )
    parser.add_argument("--position-tolerance", type=float, default=0.5, help="Accepted position error in degrees")
    parser.add_argument("--move-timeout", type=float, default=8.0, help="Timeout per test in seconds")
    parser.add_argument("--poll-ms", type=int, default=50, help="RPP polling period in ms")
    parser.add_argument("--settle-ms", type=int, default=500, help="Pause between tests in ms")
    parser.add_argument("--output", default="results/move_tests.xlsx", help="Output XLSX path")
    args = parser.parse_args()

    axes = parse_csv_numbers(args.axes, int)
    positions = parse_csv_numbers(args.positions, float)
    velocity_percent = parse_csv_numbers(args.velocity_percent, float)
    acceleration_percent = parse_csv_numbers(args.acceleration_percent, float)

    tests = generate_test_cases(
        axes=axes,
        positions_deg=positions,
        velocity_percent_values=velocity_percent,
        acceleration_percent_values=acceleration_percent,
    )

    total_tests = len(tests)
    print(f"Prepared {total_tests} tests")
    print(f"Axes: {axes}")

    aborted = False

    def handle_sigint(_sig, _frame):
        nonlocal aborted
        aborted = True
        print("\\nStop requested. Finishing current test and saving partial results...")

    signal.signal(signal.SIGINT, handle_sigint)

    client = SerialRobotClient(port=args.port, baud=args.baud)
    results: List[TestResult] = []
    try:
        idx = 0
        cnt = 0
        while idx < total_tests:
            print(f"[{(idx+1) // 2}/{(total_tests)//2}] Running tests for Axis {tests[idx].axis}...")
            print(f"{tests[idx].target_velocity_percent} % speed, {tests[idx].target_acceleration_percent} % accel\n")
            for i in range(5):
                case1 = tests[idx]
                case2 = tests[idx + 1] if (idx + 1) < total_tests else None
                result = run_single_test(client=client,
                                            case=case1,
                                            position_tolerance_deg=args.position_tolerance,
                                            move_timeout_s=args.move_timeout,
                                            poll_period_s=args.poll_ms / 1000.0,
                                            settle_delay_s=args.settle_ms / 1000.0,
                )
                results.append(result)
                print(f"    -> status={result.status}, "
                        f"result_position={result.result_position}, move_time_ms={result.result_move_time_ms:.1f}"
                )
                if case2 is not None:
                    result = run_single_test(client=client,
                                                case=case2,
                                                position_tolerance_deg=args.position_tolerance,
                                                move_timeout_s=args.move_timeout,
                                                poll_period_s=args.poll_ms / 1000.0,
                                                settle_delay_s=args.settle_ms / 1000.0,
                )
            idx = idx + 2

    #     for idx, case in enumerate(tests, start=1):
    #         if aborted:
    #             break

    #         print(
    #             f"[{idx}/{total_tests}] Axis {case.axis}: pos={case.target_position_deg:+g} deg, "
    #             f"vel={case.target_velocity_percent:g}%, acc={case.target_acceleration_percent:g}%"
    #         )

    #         result = run_single_test(
    #             client=client,
    #             case=case,
    #             position_tolerance_deg=args.position_tolerance,
    #             move_timeout_s=args.move_timeout,
    #             poll_period_s=args.poll_ms / 1000.0,
    #             settle_delay_s=args.settle_ms / 1000.0,
    #         )

    #         results.append(result)
    #         print(
    #             f"    -> status={result.status}, "
    #             f"result_position={result.result_position}, move_time_ms={result.result_move_time_ms:.1f}"
    #         )
    finally:
        client.close()

    output_path = Path(args.output)
    write_results_xlsx(output_path, results)
    print(f"Saved {len(results)} results to: {output_path}")

    if aborted:
        return 130
    return 0


if __name__ == "__main__":
    sys.exit(main())
