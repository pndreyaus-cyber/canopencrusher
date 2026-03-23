from serialRobotClient import SerialRobotClient
from ik1 import IkParameters, calc_ik_simple, calculate_angles
import math
import argparse


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
    angles = [((0.0, 0.58, 0.275), 0),
              ((0.10, 0.48, 0.375), 0),
              ((0.0, 0.38, 0.275), 0),
              ((0.10, 0.38, 0.375), 0)]
    # (0.0, 0.58, 0.275)
    # (0.10, 0.48, 0.375)
    # (0.10, 0.48, 0.375)
    # (0.0, 0.38, 0.275)
    # (0.10, 0.38, 0.375)
    # (0.0, 0.58, 0.275)
    for a in angles:
        ra = a[0]
        action = a[1]
        joint_positions = calculate_angles(*ra, 0.286, 0.049, 0.370, 0.370, 0.115)
        print(
            list(map(lambda x: x * 180 / math.pi, joint_positions))
        )  # Convert radians to degrees

        command = "MAP JA{:.2f} JB{:.2f} JC{:.2f} JD{:.2f} SP0.1 AC0.01".format(
            180 * joint_positions[0] / math.pi + 90,
            180 * joint_positions[1] / math.pi,
            180 * joint_positions[2] / math.pi,
            min(180 * joint_positions[3] / math.pi, 110),
        )
        print(f"Generated command: {command}")
        #do_run = input("Run command? (y/n):")
        do_run = "y"
        if do_run == "y":
            client.send_command(command)
            reply = client.wait_for_prefix("MAP", 30)
            print("reply:", reply)
    
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
