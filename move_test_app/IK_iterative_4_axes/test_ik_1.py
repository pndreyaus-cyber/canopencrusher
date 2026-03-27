from serialRobotClient import SerialRobotClient
#from ik1 import IkParameters, calc_ik_simple, calculate_angles
import math
import argparse
from typing import List
import numpy as np
from numpy import matmul
from ikpy.chain import Chain

transform_matrix = [[0.982685, -0.02253, 0.022949, 30.20585], # Changed -21.7568 to -17.7568
                    [-0.00843, 0.939357, 0.000573, 559.0083], # Changed 542.5955 to 546.5955
                    [-0.00948, -0.01456, 1.032103, 281.2634]]


def calculate_real_position(position_theory: tuple[float, float, float]) -> tuple[float, float, float]:
    return tuple(matmul(transform_matrix, [*position_theory, 1]))

def test_ik_1(port, baud, axes):
    client = SerialRobotClient(port=port, baud=baud, axes_num=axes)

    chain = Chain.from_urdf_file(
        "robot_4.urdf",
        active_links_mask=[False, True, True, True, True, True]
    )

    # positions_actions = [((0, 307, 792.2), 0)]
    # positions_actions = [([0+np.cos(i)*50, 414+np.sin(i)*50, 534], 0) for i in np.linspace(0, 6*math.pi, 48)]
    
    target_orient = [0, 0, -1]
    target_orient_axis = "Z"
    target_pos = [-0.5, 0, 0.534]
    angles = [0, 0, 0, 3.14159/4, 3.14159/4, 0]
    target_positions = [[-0.4, 0.30, 0.534], [-0.4, -0.30, 0.534], [-0.6, -0.30, 0.534], [-0.6, 0.30, 0.534], [-0.4, 0.30, 0.534], [-0.4, -0.30, 0.534], [-0.6, -0.30, 0.534], [-0.6, 0.30, 0.534]]
    
    for target_pos in target_positions:

        prev_angles = angles
        angles = chain.inverse_kinematics(
            target_position=target_pos,
            # target_orientation=target_orient,
            # orientation_mode=target_orient_axis,
            initial_position=prev_angles
        )

        print(angles)

        #position_theory = position_action[0]
        #position = calculate_real_position(position_theory)
        #position = position_theory
        #print("Real position: ", position)
        #action = position_action[1]
        #joint_positions = calculate_angles(*position, 49, 286, 372, 365.5, 115.5)
        # print(
        #    list(map(lambda x: x * 180 / math.pi, joint_positions))
        # )  # Convert radians to degrees

        command = "MAP JA{:.2f} JB{:.2f} JC{:.2f} JD{:.2f} SP0.1 AC0.00".format(
            180 * angles[1] / math.pi,
            180 * angles[2] / math.pi,
            180 * angles[3] / math.pi,
            min(180 * angles[4] / math.pi, 110),
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
