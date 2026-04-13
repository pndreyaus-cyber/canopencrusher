"""
You can choose different IK solvers
The only requirement is for this solver to have a function called ik,
     which will take the target position from a given file and will output angles for robot's motors
"""
import time

from SerialRobotClient import SerialRobotClient
from Point import Point, SpeedAcc
from CoordinateTransform import CoordinateTransform
from GeometryKS_4Axes_EndEffectorVertical import GeometryKS_4Axes_EndEffectorVertical
from GeometryIK_4Axes_AnyAngleOfEndEffector import GeometryIK_4Axes_AnyAngleOfEndEffector

import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from parser import create_parser

"""
File example:
0.0, 0.0, 0.0
10.1, -12.0, 2.3
0.0, -4.3, 3.2
"""


def run_moves_in_theory_coordinates(port: str, baud: int, file_path: str, solver_name: str = "GeometryKS_4Axes_EndEffectorVertical", ask_before_send: bool = True):
    client = SerialRobotClient(port=port, baud=baud, axes_num=4)

    solver = None
    match solver_name:
        case "GeometryKS_4Axes_EndEffectorVertical":
            solver = GeometryKS_4Axes_EndEffectorVertical(du=49, dv=286, l1=372, l2=305.5, l3=115.5)
        case "GeometryIK_4Axes_AnyAngleOfEndEffector":
            solver = GeometryIK_4Axes_AnyAngleOfEndEffector()
    
    if solver is None:
        print("Solver name could not be found. Exiting...")
        return

    positions = []
    with open(file_path, "r") as f:
        for line in f:
            if line.startswith("#") or not line.strip(): # Skip comments and empty lines
                continue
            parts = line.strip().split(",")
            if len(parts) != 3:
                print(f"Invalid line in file: {line}")
                continue
            x, y, z = parts
            positions.append(Point(float(x), float(y), float(z)))

    for position_robot in positions:
        print("Position robot: ", position_robot)

        joint_angles = solver.ik(position_robot)
        print("Joint angles (degrees): ", client.radians_to_degrees(joint_angles))

        reply = client.move_to_radians(
            joint_angles, SpeedAcc(0.15, 0.01), ask_before_send=ask_before_send
        )
        print("Move command reply: ", reply)

        fk_position = solver.fk(*joint_angles)
        print("FK position: ", fk_position)

        time.sleep(2)


if __name__ == "__main__":
    parser = create_parser(
        "Moving to positions in robot coordinates. Position are read from a given file"
    )

    parser.add_argument("--file", required=True, help="File with positions and actions")
    parser.add_argument("--solver", type=str, default="GeometryKS_4Axes_EndEffectorVertical", help="IK solver to use")
    parser.add_argument("--ask-before-send", action="store_true", help="Ask for confirmation before sending each command")
    args = parser.parse_args()
    run_moves_in_theory_coordinates(args.port, args.baud, args.file, args.solver, args.ask_before_send)
