from Vision.vision_centroid_lib.detector import WhiteObjectCentroidDetector
from Movement.SerialRobotClient import SerialRobotClient
from parser import create_parser
from Movement.CoordinateTransform import CoordinateTransform
from Movement.Point import Point, SpeedAcc

import cv2
import time

CUBE_HEIGHT = 25 # mm

def check_has_cubes_stability(detector, cap, tries=3):
    cube_count = 0
    for i in range(tries):
        ret, frame = cap.read()
        has_objects = detector.has_objects(frame)
        if has_objects:
            centroids_mm = detector.get_centroids_mm(frame)
            cube_count += len(centroids_mm)
    return cube_count >= tries

def run(port, baud, axes, transform_path: str, move_conveyor: bool =False,  ask_before_send: bool = True):
    client = SerialRobotClient(port=port, baud=baud, axes_num=axes)
    
    client.set_geometryKS4Axes(du=49, dv=286, l1=372, l2=305.5, l3=115.5)

    transform = CoordinateTransform()
    transform.load_transform_matrix_from_file(transform_path)

    detector = WhiteObjectCentroidDetector(
        model_path="Vision/Model/white_cube_yolo26s_finetune2_best.pt",
        camera_calibration_path="Vision/camera_calibration.npz",
        homography_path="Vision/calibration_4pt_homography.npz",
        conf_threshold=0.5,
        crop=(40, 430, 0, 640),
    )
    
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError("Не удалось открыть камеру")
    
    try:
        if move_conveyor:
            client.send_command("CON")
            reply = client.wait_for_prefix("CON", 5)
            client.check_reply(reply)

        successfull_cubes = 0
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            if not check_has_cubes_stability(detector, cap):
                print("No cubes detected, skipping...")
                continue

            print("Cubes detected, processing...")
            if move_conveyor:
                client.send_command("COF")
                reply = client.wait_for_prefix("COF", 5)
                client.check_reply(reply)

            ret, frame = cap.read()
            centroids_mm = detector.get_centroids_mm(frame)
            result = detector.process_frame(frame)
            cv2.imshow("Annotated", result.annotated_frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

            print(f"Centroids (mm): {centroids_mm}")
            
            for centroid in centroids_mm:
                print(f"Centroid {centroid} mm")

                if input("Process this cube? (y/n):") != "y":
                    continue

                cube_position_theory = Point(centroid[0], centroid[1], CUBE_HEIGHT)
                result = client.geometry_ik_PAD(
                    transform.theoretical_to_robot_coordinates(cube_position_theory),
                    cube_hover_dist=30,
                    move_sa = SpeedAcc(0.15, 0.01),
                    cube_pick_delta = 5,
                    up_and_down_sa= SpeedAcc(0.1, 0.005),
                    drop_position_robot_coordinates=Point(50.0, 430.0, 340  + successfull_cubes * CUBE_HEIGHT),
                    drop_hover_dist=30,
                    move_immediately_to_pick_position=True
                )
                
                if result:
                    successfull_cubes += 1
                    print("Successfully moved cube. Total cubes moved: ", successfull_cubes)
                else:
                    print("Failed to move cube at position: ", centroid)
                
            print("Parsed all cubes")
            if move_conveyor:
                print("Turning on conveyor for next cubes...")
                client.send_command("CON")
                reply = client.wait_for_prefix("CON", 5)
                client.check_reply(reply)


    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    parser = create_parser("Pick and Drop using camera detection")
    parser.add_argument("--move_conveyor", action="store_true", help="Whether to move the conveyor or not")
    parser.add_argument("--transform", required=True, help="Path to transformation matrix")


    args = parser.parse_args()
    run(args.port, args.baud, args.axes, args.transform, move_conveyor=args.move_conveyor)

