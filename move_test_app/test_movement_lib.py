from serialRobotClient import SerialRobotClient
from movement_lib import make_move
import argparse

def run_test(port, baud, axes):
    client = SerialRobotClient(port=port, baud=baud, axes_num=axes)
    cube_position_theory_plane = (30, 80)
    make_move(client,
              cube_position_theory_plane,
              cube_hover_dist=30,
              cube_pick_dist=5,
              drop_point_theory=(-100, -220, -110),
              drop_hover_dist=30,
              drop_place_dist=20,
              cubes_filtered=1)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Testing Pick and Place Movement")

    parser.add_argument("--port", required=True, help="Serial port, e.g. COM7")
    parser.add_argument("--axes", required=True, type=int, help="Number of axes to use")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    args = parser.parse_args()
    run_test(args.port, args.baud, args.axes)
