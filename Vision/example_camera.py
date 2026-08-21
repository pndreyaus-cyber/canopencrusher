import cv2

from vision_centroid_lib.detector import WhiteObjectCentroidDetector

def main() -> None:
    detector = WhiteObjectCentroidDetector(
        model_path="Model/white_cube_yolo26s_finetune2/white_cube_yolo26s_finetune2/weights/best.pt",
        camera_calibration_path="camera_calibration.npz",
        homography_path="calibration_4pt_homography.npz",
        conf_threshold=0.4,
        crop=(40, 430, 0, 640),
    )

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError("Не удалось открыть камеру")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            has_objects = detector.has_objects(frame)
            centroids_mm = detector.get_centroids_mm(frame)
            result = detector.process_frame(frame)

            print(f"has_objects = {int(has_objects)}")
            print(f"centroids_mm = {centroids_mm}")

            cv2.imshow("Annotated", result.annotated_frame)
            cv2.imshow("Processed", result.processed_frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
