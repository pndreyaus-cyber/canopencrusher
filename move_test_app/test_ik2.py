from serialRobotClient import SerialRobotClient, run_commands
import math
import argparse
from ik2 import solve_robot
from ik1 import calculate_angles
import time


def test_ik_2(port, baud, axes):
    client = SerialRobotClient(port=port, baud=baud, axes_num=axes)
    positions = [(-0.064, 0.395, 0.575)]
    for pos in positions:
        angles = calculate_angles(*pos)
        command = "MAP JA{:.2f} JB{:.2f} JC{:.2f} JD0.0 JE{:.2f} SP0.1 AC0.005".format(
            180 * angles[0] / math.pi,
            180 * angles[1] / math.pi,
            180 * angles[2] / math.pi,
            min(-180 * angles[3] / math.pi, 110),
        )
        print(f"Generated command: {command}")
        run_commands([(command, "MAP")], client, move_timeout_s=20)
        time.sleep(2)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Testing IK1")

    parser.add_argument("--port", required=True, help="Serial port, e.g. COM7")
    parser.add_argument("--axes", required=True, type=int, help="Number of axes to use")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    args = parser.parse_args()
    test_ik_2(args.port, args.baud, args.axes)
