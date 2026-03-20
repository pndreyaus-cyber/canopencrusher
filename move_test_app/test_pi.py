"""
I want to
1) Repeat the same movement (sequesnce of MAP commands)
2) After repeating 3-4 times, I want to change 1 of 4 parameters. But how do i choose it? 
By asking the user for input? Or by just iterating over available values?
Ok, let's do both, but add the argument to the program, which will control this stuff
"""

import argparse
from serialRobotClient import SerialRobotClient

movement_commands = [("MAP JA-180.0 JB120.0 JC0.0 JD0.0 SP0.1 AC0.02", "MAP"), ("MAP JA-180.0 JB0.0 JC0.0 JD0.0 SP0.1 AC0.02", "MAP")]
REPEAT_TIMES = 4

def run_in_range(client: SerialRobotClient):
    while(1):
        for _ in range(REPEAT_TIMES):
            for move_command in movement_commands:
                client.send_command(move_command[0])
                client.wait_for_prefix(move_command[1], 10)

def run_user_input(axis_check: int, axes, port, baud):
    client = SerialRobotClient(port=port, baud=baud, axes_num=axes)
    while(1):
        client.send_command("RPI J" + str(chr(ord('A') + axis_check - 1)))
        reply = client.wait_for_prefix("RPI", 30)
        if reply is None:
            print("Cannot run test. COuld not read current PI value")
            return
        print("Running test for: " + reply)

        for _ in range(REPEAT_TIMES):
            for move_command in movement_commands:
                client.send_command(move_command[0])
                client.wait_for_prefix(move_command[1], 30)
        new_parameter = input("Enter new parameter value in form (JA P1 V2000): ")
        client.send_command("PIC " + new_parameter)
        reply = client.wait_for_prefix("PIC", 30)
        print("Parameter set status: " + ("None" if reply is None else reply))

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="PI controller setter"
    )
    parser.add_argument("--port", required=True, help="Serial port, e.g. COM7")
    parser.add_argument("--axis_check", required=True, type=int, help="What axis to check")
    parser.add_argument("--axes", required=True, type=int, help="Number of axes to use")
    parser.add_argument("-i", "--user-input", action="store_true")

    parser.add_argument("-c", "--check-replies", action="store_true")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument(
        "--position-tolerance",
        type=float,
        default=10,
        help="Accepted position error in steps",
    )
    parser.add_argument(
        "--reply-timeout", type=float, default=10, help="Reply timeout in seconds"
    )
    args = parser.parse_args()

    if args.user_input:
        print("Running user input...")
        run_user_input(args.axis_check, args.axes, args.port, args.baud)