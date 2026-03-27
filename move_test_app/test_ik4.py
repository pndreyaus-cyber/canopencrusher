from serialRobotClient import SerialRobotClient
from ik1 import IkParameters, calc_ik_simple, calculate_angles
import math
import argparse
from typing import List
import numpy as np
from numpy import matmul

transform_matrix = [[0.982685, -0.02253, 0.022949, 30.20585], # Changed -21.7568 to -17.7568
                    [-0.00843, 0.939357, 0.000573, 559.0083], # Changed 542.5955 to 546.5955
                    [-0.00948, -0.01456, 1.032103, 281.2634]]


def calculate_real_position(position_theory: tuple[float, float, float]) -> tuple[float, float, float]:
    return tuple(matmul(transform_matrix, [*position_theory, 1]))

def test_ik_1(port, baud, axes):
    client = SerialRobotClient(port=port, baud=baud, axes_num=axes)
    # angles = [((-0.023, 0.40, 0.290), 0),
    #           ((-0.023, 0.40, 0.284), 1),
    #           ((-0.023, 0.40, 0.290), 0),
    #           ((0.145, 0.40, 0.290), 0),
    #           ((0.145, 0.40, 0.284), 2),
    #           ((0.145, 0.40, 0.290), 0),
    #           ((-0.023, 0.40, 0.290), 0),
    #           ((0.145, 0.40, 0.290), 0),
    #           ((0.145, 0.40, 0.284), 1),
    #           ((0.145, 0.40, 0.290), 0),
    #           ((-0.023, 0.40, 0.290), 0),
    #           ((-0.023, 0.40, 0.284), 2),  ]
    # angles = [((0.0, 0.58, 0.275), 0),
    #           ((0.10, 0.48, 0.375), 0),
    #           ((0.0, 0.38, 0.275), 0),
    #           ((0.10, 0.38, 0.375), 0)]
    # (0.0, 0.58, 0.275)
    # (0.10, 0.48, 0.375)
    # (0.10, 0.48, 0.375)
    # (0.0, 0.38, 0.275)
    # (0.10, 0.38, 0.375)
    # (0.0, 0.58, 0.275)
    # positions_actions = [((0, 307, 792.2), 0)]
    positions_actions = [([0+np.cos(i)*50, 414+np.sin(i)*50, 534], 0) for i in np.linspace(0, 6*math.pi, 48)]
    
    # target = [0, 365+49, 534]
    #target = [0, 307, 792.2]

    for position_action in positions_actions:
        position_theory = position_action[0]
        #position = calculate_real_position(position_theory)
        position = position_theory
        #print("Real position: ", position)
        #action = position_action[1]
        joint_positions = calculate_angles(*position, 49, 286, 372, 365.5, 115.5)
        print(
           list(map(lambda x: x * 180 / math.pi, joint_positions))
        )  # Convert radians to degrees

        command = "MAP JA{:.2f} JB{:.2f} JC{:.2f} JD{:.2f} SP0.15 AC0.02".format(
            180 * joint_positions[0] / math.pi,
            180 * joint_positions[1] / math.pi,
            180 * joint_positions[2] / math.pi,
            min(180 * joint_positions[3] / math.pi, 110),
        )
        #angles = solver.solve(position_theory, 90, 0)
        #print(angles)

        # command = "MAP JA{:.2f} JB{:.2f} JC{:.2f} JD{:.2f} SP0.1 AC0.02".format(
        #     angles[0],
        #     angles[1],
        #     angles[2],
        #     angles[3],
        # )
        print(f"Generated command: {command}")
        #do_run = input("Run command? (y/n):")
        do_run = "y"
        if do_run == "y":
            client.send_command(command)
            reply = client.wait_for_prefix("MAP", 30)
            print("reply:", reply)
    
        action = 3
        if action == 1:
            client.send_command("GRB")
            reply = client.wait_for_prefix("GRB", 30)
            print("reply:", reply)
        elif action == 2:
            client.send_command("LGO")
            reply = client.wait_for_prefix("LGO", 30)
            print("reply:", reply)
        
        # run_commands([(command, "MAP")], client, move_timeout_s=20)
    # run_commands([("MAP " + " ".join(f"{chr(ord('A') + i)}{joint_positions[i]:.2f}" for i in range(len(joint_positions))), "MAP")], client, move_timeout_s=5)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Testing IK1")

    parser.add_argument("--port", required=True, help="Serial port, e.g. COM7")
    parser.add_argument("--axes", required=True, type=int, help="Number of axes to use")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    args = parser.parse_args()
    test_ik_1(args.port, args.baud, args.axes)
