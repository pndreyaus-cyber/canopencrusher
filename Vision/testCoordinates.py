import cv2
import numpy as np

# Crop, соответствующий рабочей программе
CROP_Y1 = 40
CROP_Y2 = 430
CROP_X1 = 0
CROP_X2 = 640

CAMERA_ID = 0

# Имена файлов
CAMERA_CALIB_FILE = "camera_calibration.npz"
H_CALIB_FILE = "calibration_4pt_homography.npz"


def draw_clicked_points(img, points):
    for i, (x, y, X_mm, Y_mm) in enumerate(points):
        cv2.circle(img, (x, y), 5, (0, 0, 255), -1)
        text = f"{i}: ({X_mm:.2f}, {Y_mm:.2f})"
        cv2.putText(
            img,
            text,
            (x + 8, y - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 0, 255),
            1,
            cv2.LINE_AA,
        )


def preprocess_like_runtime(frame, camera_matrix, dist_coeffs):
    """
    Делает ТОЧНО ту же предобработку, что и рабочая программа:
    1) undistort без new_camera_matrix
    2) crop [40:430, 0:640]
    """
    undistorted = cv2.undistort(frame, camera_matrix, dist_coeffs)

    h, w = undistorted.shape[:2]
    x1 = max(0, CROP_X1)
    y1 = max(0, CROP_Y1)
    x2 = min(w, CROP_X2)
    y2 = min(h, CROP_Y2)

    if x2 <= x1 or y2 <= y1:
        raise RuntimeError("Crop size is not correct.")

    cropped = undistorted[y1:y2, x1:x2]
    return undistorted, cropped


def draw_text_block(img, lines, x=10, y=25, dy=28, color=(0, 255, 0)):
    for i, line in enumerate(lines):
        cv2.putText(
            img,
            line,
            (x, y + i * dy),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            color,
            2,
            cv2.LINE_AA,
        )


def pixel_to_mm(x, y, H_matrix):
    pt = np.array([x, y, 1.0], dtype=np.float32)
    res = H_matrix @ pt
    if abs(res[2]) < 1e-9:
        raise ValueError("Wrong matrix H")
    X_mm = res[0] / res[2]
    Y_mm = res[1] / res[2]
    return float(X_mm), float(Y_mm)


def mouse_callback(event, x, y, flags, param):
    H_matrix, clicked_points = param
    if event == cv2.EVENT_LBUTTONDOWN:
        if H_matrix is None:
            return

        try:
            X_mm, Y_mm = pixel_to_mm(x, y, H_matrix)
            clicked_points.append((x, y, X_mm, Y_mm))
            print(f"Click: px=({x}, {y}) -> mm=({X_mm:.2f}, {Y_mm:.2f})")
        except Exception as e:
            print("Error of point transforming:", e)

    elif event == cv2.EVENT_RBUTTONDOWN:
        # Правая кнопка удаляет последнюю точку
        if clicked_points:
            clicked_points.pop()
    return clicked_points


def testCoordinates(H_matrix, cap: cv2.VideoCapture, camera_matrix, dist_coeffs):
    # ============================================
    # ЭТАП 3. СРАЗУ ТЕСТИРОВАНИЕ
    # ============================================

    clicked_points = []

    print("\nЭтап 3: тестирование")
    print("ЛКМ - добавить точку")
    print("ПКМ - удалить последнюю точку")
    print("C - очистить все точки")
    print("Q - выход")

    cv2.namedWindow("Runtime test")
    cv2.setMouseCallback("Runtime test", mouse_callback, param=(H_matrix, clicked_points))

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Не удалось получить кадр.")
            continue

        undistorted_full, runtime_view = preprocess_like_runtime(
            frame, camera_matrix, dist_coeffs
        )

        test_view = runtime_view.copy()
        draw_clicked_points(test_view, clicked_points)

        lines = [
            "Phase 3: live test",
            "Press LMB to choose point",
            "Press RMB to delete last point",
            "Press C to clean all points",
            "Press Q to exit",
        ]
        draw_text_block(test_view, lines, color=(255, 255, 0))

        cv2.imshow("Orig", frame)
        cv2.imshow("Runtime test", test_view)
        cv2.imshow("Undistorted full", undistorted_full)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break
        elif key == ord("c"):
            clicked_points = []


if __name__ == "__main__":
    camera_calib_file = np.load(CAMERA_CALIB_FILE)
    # Access camera_matrix
    camera_matrix = camera_calib_file["camera_matrix"]
    # Access dist_coeffs
    dist_coeffs = camera_calib_file["dist_coeffs"]

    h_calib_file = np.load(H_CALIB_FILE)
    H = h_calib_file["H"]

    cap = cv2.VideoCapture(CAMERA_ID)
    testCoordinates(H, cap, camera_matrix, dist_coeffs)

    cap.release()
    cv2.destroyAllWindows()
