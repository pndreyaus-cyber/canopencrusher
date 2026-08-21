import argparse

def create_parser(app_name):
    parser = argparse.ArgumentParser(description=app_name)
    parser.add_argument("--port", required=True, help="Serial port, e.g. COM7")
    parser.add_argument("--axes", required=True, type=int, help="Number of axes to use")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument("--timeout", type=float, help="Timeout for serial")
    
    return parser