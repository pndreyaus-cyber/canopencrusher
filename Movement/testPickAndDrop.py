from SerialRobotClient import SerialRobotClient

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from parser import create_parser
from Point import Point, SpeedAcc

cube_positions = [Point(0, 0, 0), Point(51.83, 24.5, 0)] # In robot coordinates

def run_test(port, baud, axes):
    client = SerialRobotClient(port=port, baud=baud, axes_num=axes)

    client.set_geometryKS4Axes(
        du = 49,
        dv = 286,
        l1 = 372,
        l2 = 365.5,
        l3 =  115.5
    )

    for cube_position in cube_positions:
        print(f"Testing pick and drop for cube at position: {cube_position}")
        result = client.geometry_ik_PAD(
            cube_position_robot_coordinates=cube_position,
            cube_hover_dist = 30,
            move_sa=SpeedAcc(0.3, 0.01),
            cube_pick_delta=10,
            up_and_down_sa=SpeedAcc(0.07, 0.005),
            drop_position_robot_coordinates=Point(100, 100, 100),
            drop_hover_dist=30,
            ask_before_steps=True,
            move_immediately_to_pick_position=True,
        )
        if result:
            print("Pick and drop successful!")
        else:
            print("Pick and drop failed!")


if __name__ == "__main__":
    parser = create_parser("Test pick and drop movement")
    args = parser.parse_args()
    run_test(args.port, args.baud, args.axes)
