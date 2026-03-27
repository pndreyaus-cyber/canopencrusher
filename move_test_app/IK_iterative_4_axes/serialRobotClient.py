from typing import Iterable, List, Optional, Tuple
import serial  # type: ignore
import time
from math import pi

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

    def move_to(self, joint_angles_rad: tuple[float, float, float, float], speed: float, acc: float) -> None:
        joint_angles_deg = tuple(angle * 180 / pi for angle in joint_angles_rad)
        command = f"MAP JA{joint_angles_deg[0]:0.3f} JB{joint_angles_deg[1]:0.3f} JC{joint_angles_deg[2]:0.3f} JD{joint_angles_deg[3]:0.3f} SP{speed:0.3f} AC{acc:0.3f}"
        print("Command to send: ", command)
        y = input("Send command? (y/n):")
        if y.lower() == "y":
            self.send_command(command)

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