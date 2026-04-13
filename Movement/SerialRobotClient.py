import math
from typing import Iterable, List, Optional, Tuple, LiteralString
import serial  # type: ignore
import time
from math import pi
try: 
    from .GeometryKS_4Axes_EndEffectorVertical import GeometryKS_4Axes_EndEffectorVertical
    from .Point import Point, SpeedAcc
except ImportError:
    from GeometryKS_4Axes_EndEffectorVertical import GeometryKS_4Axes_EndEffectorVertical
    from Point import Point, SpeedAcc

class SerialRobotClient:
    def __init__(
        self,
        port: str,
        baud: int,
        axes_num: int,
        timeout_s: float = 5,
    ) -> None:
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=timeout_s)
        self.axes_num = axes_num
        self.timeout = timeout_s

        self.geometry_set = False

        self.current_joint_angles = None


    def close(self) -> None:
        if self.ser and self.ser.is_open:
            self.ser.close()

    def clear_input_buffer(self) -> None:
        self.ser.reset_input_buffer()

    def send_command(self, command: str) -> None:
        payload = f"{command}\n".encode("ascii", errors="ignore")
        self.ser.write(payload)
        self.ser.flush()

    def move_to_degrees(self, joint_angles: List[float], sa: SpeedAcc, ask_before_send: bool = False) -> Optional[str]:
        command = f"MA"
        # joint_angles[0] -=180
        # joint_angles[1] = -joint_angles[1]
        # joint_angles[2] = -joint_angles[2]
        # joint_angles[3] = -joint_angles[3]
        for i, angle in enumerate(joint_angles):
            command += f" J{chr(ord('A') + i)}{ + angle:0.4f}"
        command += f" SP{sa.speed:0.4f} AC{sa.acc:0.4f}"
        print("Command to send: ", command)

        if ask_before_send:
            y = input("Send command? (y/n):")
            if y.lower() == "y":
                self.send_command(command)
            else:
                print("Command not sent")
                return None
        else:
            self.send_command(command)


        reply = self.wait_for_prefix("MA")
        if self.check_reply(reply):
            self.current_joint_angles = joint_angles
        
        return reply

    def radians_to_degrees(self, joint_angles: List[float]) -> List[float]:
        return [angle * 180 / pi for angle in joint_angles]

    def move_to_radians(self, joint_angles: List[float], sa: SpeedAcc, ask_before_send: bool = False) -> Optional[str]:
        return self.move_to_degrees(self.radians_to_degrees(joint_angles), sa, ask_before_send)

    def read_line(self) -> Optional[str]:
        raw = self.ser.readline()
        if not raw:
            return None
        try:
            line = raw.decode("utf-8", errors="ignore").strip()
        except UnicodeDecodeError:
            return None
        return line if line else None

    def wait_for_prefix(self, prefix_str: str, timeout_s: float = 0) -> Optional[str]:
        if timeout_s == 0:
            timeout_s = self.timeout

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
    
    def set_geometryKS4Axes(self, du: float, dv: float, l1: float, l2: float, l3: float):
        self.geometry = GeometryKS_4Axes_EndEffectorVertical(du, dv, l1, l2, l3)
        self.geometry_set = True
    
    def check_reply(self, reply: str | None) -> bool:
        if reply is None:
            print("No reply received")
            return False
        blocks = reply.split()
        if blocks[1] != "OK":
            print(f"Reply : '{reply}' failed")
            return False
        return True

    def geometry_ik_PAD(self,
                  cube_position_robot_coordinates: Point,
                  cube_hover_dist: float,
                  move_sa: SpeedAcc,
                  cube_pick_delta: float,
                  up_and_down_sa: SpeedAcc,
                  drop_position_robot_coordinates: Point,
                  drop_hover_dist: float, 
                  current_robot_coordinates: Point = Point(0, 0, 0),
                  ask_before_steps: bool = False,
                  move_immediately_to_pick_position: bool = False
                  ): # Pick And Drop

        if not self.geometry_set:
            print("Geometry parameters not set! Call set_geometryKS4Axes first")
            return

        if not move_immediately_to_pick_position:
            # Step 1 -- move up at the current position
            print("Step 1 -- move up at the current position")
            angles = self.geometry.ik(Point(current_robot_coordinates.x,
                                    current_robot_coordinates.y,
                                    cube_position_robot_coordinates.z + cube_hover_dist))

            self.move_to_radians(angles, up_and_down_sa, ask_before_steps)
            reply = self.wait_for_prefix("MAP", 20.0)
            if not reply:
                return False
        
        # Step 2 -- move above the cube
        print("Step 2 -- move above the cube")
        angles = self.geometry.ik(Point(cube_position_robot_coordinates.x,
                                  cube_position_robot_coordinates.y,
                                  cube_position_robot_coordinates.z + cube_hover_dist))

        reply = self.move_to_radians(angles, move_sa, ask_before_steps)
        print("Moved successfully above the cube, waiting for reply...")
        #reply = self.wait_for_prefix("MAP", 20.0)
        if not reply:
            return False
        
        # Step 3 -- move down to the cube
        print("Step 3 -- move down to the cube")

        angles = self.geometry.ik(Point(cube_position_robot_coordinates.x,
                                  cube_position_robot_coordinates.y,
                                  cube_position_robot_coordinates.z - cube_pick_delta))
        reply = self.move_to_radians(angles, up_and_down_sa, ask_before_steps)
        #reply = self.wait_for_prefix("MAP", 20.0)
        if not reply:
            return False
        
        # Step 4 -- pick up the cube
        print("Step 4 -- pick up the cube")
        self.send_command("GRB")
        reply = self.wait_for_prefix("GRB", 20.0)
        if not reply:
            return False
        print("Successfully picked up the cube")
        time.sleep(0.1)

        # Step 5 -- move up with the cube
        print("Step 5 -- move up with the cube")
        angles = self.geometry.ik(Point(cube_position_robot_coordinates.x,
                                  cube_position_robot_coordinates.y,
                                  cube_position_robot_coordinates.z + cube_hover_dist))
        reply = self.move_to_radians(angles, up_and_down_sa, ask_before_steps)
        #reply = self.wait_for_prefix("MAP", 20.0)
        if not reply:
            return False
        print("Successfully moved up with the cube")
        
        # Step 6 -- move above the drop position
        print("Step 6 -- move above the drop position")
        angles = self.geometry.ik(Point(drop_position_robot_coordinates.x,
                                  drop_position_robot_coordinates.y,
                                  drop_position_robot_coordinates.z + drop_hover_dist))
        reply = self.move_to_radians(angles, move_sa, ask_before_steps)
        #reply = self.wait_for_prefix("MAP", 20.0)
        if not reply:
            return False
        print("Successfully moved above the drop position")

        # Step 7 -- move down to the drop position
        print("Step 7 -- move down to the drop position")
        angles = self.geometry.ik(Point(drop_position_robot_coordinates.x,
                                  drop_position_robot_coordinates.y,
                                  drop_position_robot_coordinates.z))
        reply = self.move_to_radians(angles, up_and_down_sa, ask_before_steps)
        #reply = self.wait_for_prefix("MAP", 20.0)
        if not reply:
            return False
        print("Successfully moved down to the drop position")
        
        # Step 8 -- drop the cube
        print("Step 8 -- drop the cube")
        self.send_command("LGO")
        reply = self.wait_for_prefix("LGO", 20.0)
        if not reply:
            return False
        print("Successfully dropped the cube")
        time.sleep(0.1)

        # Step 9 -- move up after dropping the cube
        print("Step 9 -- move up after dropping the cube")
        angles = self.geometry.ik(Point(drop_position_robot_coordinates.x,
                                  drop_position_robot_coordinates.y,
                                  drop_position_robot_coordinates.z + drop_hover_dist))
        reply = self.move_to_radians(angles, move_sa, ask_before_steps)
        #reply = self.wait_for_prefix("MAP", 20.0)
        if not reply:
            return False
        print("Successfully moved up after dropping the cube")

        print("Finished all steps successfully")
        return True


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