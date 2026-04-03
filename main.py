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
        cap, frame = cap.read()
        has_objects = detector.has_objects(frame)
        if has_objects:
            centroids_mm = detector.get_centroids_mm(frame)
            cube_count += len(centroids_mm)
    return cube_count >= tries

def run(port, baud, axes, move_conveyor=False):
    client = SerialRobotClient(port=port, baud=baud, axes_num=axes)

    transform = CoordinateTransform()
    transform.load_transform_matrix_from_file("../Movement/3axes_transform_1.csv")

    detector = WhiteObjectCentroidDetector(
        model_path="Model/white_cube_yolo26s_finetune2/white_cube_yolo26s_finetune2/weights/best.pt",
        camera_calibration_path="camera_calibration.npz",
        homography_path="calibration_4pt_homography.npz",
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
                    cube_pick_delta = 3,
                    up_and_down_sa= SpeedAcc(0.1, 0.005),
                    drop_position_robot_coordinates=Point(0, 300, 168 + successfull_cubes * CUBE_HEIGHT),
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

    args = parser.parse_args()
    run(args.port, args.baud, args.axes, move_conveyor=args.move_conveyor)

