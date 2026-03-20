from serialRobotClient import SerialRobotClient
from ik1 import IkParameters, calc_ik_simple
import math
import argparse


def test_ik_1(port, baud, axes):
    client = SerialRobotClient(port=port, baud=baud, axes_num=axes)

    ik_params = IkParameters()
    result, joint_positions = calc_ik_simple(0.32, 0.35, 0.17, ik_params)  # in meters
    print(result)
    print(joint_positions)
    print(
        list(map(lambda x: x * 180 / math.pi, joint_positions))
    )  # Convert radians to degrees
    if result:
        command = "MAP JA{:.2f} JB{:.2f} JC{:.2f} JD0.0 JE{:.2f} SP0.1 AC0.02".format(
            180 * joint_positions[0] / math.pi,
            -180 * joint_positions[1] / math.pi,
            180 * joint_positions[2] / math.pi,
            min(180 * joint_positions[3] / math.pi, 110),
        )
        print(f"Generated command: {command}")
        # run_commands([(command, "MAP")], client, move_timeout_s=20)
    # run_commands([("MAP " + " ".join(f"{chr(ord('A') + i)}{joint_positions[i]:.2f}" for i in range(len(joint_positions))), "MAP")], client, move_timeout_s=5)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Testing IK1")

    parser.add_argument("--port", required=True, help="Serial port, e.g. COM7")
    parser.add_argument("--axes", required=True, type=int, help="Number of axes to use")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    args = parser.parse_args()
    test_ik_1(args.port, args.baud, args.axes)
