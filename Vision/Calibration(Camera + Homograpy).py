import cv2
import numpy as np
from testCoordinates import preprocess_like_runtime, draw_text_block, CROP_X1, CROP_Y1, CROP_X2, CROP_Y2, testCoordinates

# ============================================
# НАСТРОЙКИ ПОД ТВОЙ РАБОЧИЙ СКРИПТ
# ============================================
CAMERA_ID = 0

# ВНУТРЕННИЕ углы шахматной доски
CHESSBOARD_SIZE = (13, 4)

# Размер клетки в мм
SQUARE_SIZE_MM = 20.0

# Минимум кадров для калибровки камеры
MIN_CALIB_FRAMES = 10

# Имена файлов
CAMERA_CALIB_FILE = "camera_calibration.npz"
H_CALIB_FILE = "calibration_4pt_homography.npz"


# ============================================
# ГЛОБАЛЬНОЕ СОСТОЯНИЕ ДЛЯ ТЕСТА МЫШКОЙ
# ============================================
clicked_points = []
current_H = None


# ============================================
# ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ
# ============================================
def build_object_points(board_size, square_size_mm):
    cols, rows = board_size
    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= square_size_mm
    return objp


def find_chessboard(gray, board_size):
    cols, rows = board_size

    flags = (
        cv2.CALIB_CB_ADAPTIVE_THRESH
        + cv2.CALIB_CB_NORMALIZE_IMAGE
        + cv2.CALIB_CB_FAST_CHECK
    )

    found, corners = cv2.findChessboardCorners(gray, board_size, flags)
    if not found:
        return False, None

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001)
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

    # Приводим порядок к каноническому:
    # первая точка = верхняя левая на изображении,
    # X растет вправо, Y растет вниз.
    corners2 = corners.reshape(rows, cols, 2)

    # Если первая строка ниже последней -> переворачиваем по вертикали
    if corners2[0, 0, 1] > corners2[-1, 0, 1]:
        corners2 = corners2[::-1, :, :]

    # Если первый столбец правее последнего -> переворачиваем по горизонтали
    if corners2[0, 0, 0] > corners2[0, -1, 0]:
        corners2 = corners2[:, ::-1, :]

    corners = corners2.reshape(-1, 1, 2)
    return True, corners


def compute_reprojection_error(
    object_points, image_points, rvecs, tvecs, camera_matrix, dist_coeffs
):
    total_error = 0.0
    total_points = 0

    for i in range(len(object_points)):
        projected, _ = cv2.projectPoints(
            object_points[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs
        )
        err = cv2.norm(image_points[i], projected, cv2.NORM_L2)
        n = len(projected)
        total_error += err * err
        total_points += n

    if total_points == 0:
        return None

    return np.sqrt(total_error / total_points)


# ============================================
# ОСНОВНАЯ ПРОГРАММА
# ============================================
def main():
    global current_H, clicked_points

    cap = cv2.VideoCapture(CAMERA_ID)
    if not cap.isOpened():
        raise RuntimeError("Can't open camera.")

    objp = build_object_points(CHESSBOARD_SIZE, SQUARE_SIZE_MM)

    object_points = []
    image_points = []
    image_size = None

    print("Этап 1: калибровка камеры")
    print("S - сохранить кадр с шахматной доской")
    print("ENTER - выполнить калибровку")
    print("Q - выход")

    # ============================================
    # ЭТАП 1. КАЛИБРОВКА КАМЕРЫ ПО ИСХОДНЫМ КАДРАМ
    # ============================================
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Не удалось получить кадр.")
            continue

        display = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        image_size = (gray.shape[1], gray.shape[0])

        found, corners = find_chessboard(gray, CHESSBOARD_SIZE)
        if found:
            cv2.drawChessboardCorners(display, CHESSBOARD_SIZE, corners, found)

        lines = [
            "Phase 1: Camera calibration",
            f"Saved frames: {len(image_points)} / minimum frames {MIN_CALIB_FRAMES}",
            "S - save frame",
            "ENTER - calibrate",
            "Q - exit",
        ]
        draw_text_block(display, lines, color=(0, 255, 0))

        cv2.imshow("Calibration", display)
        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            cap.release()
            cv2.destroyAllWindows()
            return

        elif key == ord("s"):
            if not found:
                print("Шахматная доска не найдена, кадр не сохранен.")
                continue

            object_points.append(objp.copy())
            image_points.append(corners.copy())
            print(f"Кадр сохранен. Всего: {len(image_points)}")

        elif key in (13, 10):
            if len(image_points) < MIN_CALIB_FRAMES:
                print(f"Недостаточно кадров. Нужно хотя бы {MIN_CALIB_FRAMES}.")
                continue
            break

    print("\nВыполняется калибровка камеры...")

    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        object_points, image_points, image_size, None, None
    )

    reproj_error = compute_reprojection_error(
        object_points, image_points, rvecs, tvecs, camera_matrix, dist_coeffs
    )

    print("\n=== КАЛИБРОВКА КАМЕРЫ ===")
    print("RMS:", rms)
    print("Средняя reprojection error:", reproj_error)
    print("camera_matrix:\n", camera_matrix)
    print("dist_coeffs:\n", dist_coeffs.ravel())

    np.savez(
        CAMERA_CALIB_FILE,
        camera_matrix=camera_matrix,
        dist_coeffs=dist_coeffs,
        image_width=np.array([image_size[0]], dtype=np.int32),
        image_height=np.array([image_size[1]], dtype=np.int32),
        rms=np.array([rms], dtype=np.float64),
        reprojection_error=np.array([reproj_error], dtype=np.float64),
    )

    print(f"\nПараметры камеры сохранены в {CAMERA_CALIB_FILE}")

    # ============================================
    # ЭТАП 2. РАСЧЕТ H В СИСТЕМЕ РАБОЧЕЙ ПРОГРАММЫ
    # ============================================
    print("\nЭтап 2: вычисление H")
    print("Сейчас H считается по тому же кадру, что использует рабочий код:")
    print("undistort(frame, camera_matrix, dist_coeffs) + crop")
    print("Покажи шахматную доску в рабочей плоскости.")
    print("H - вычислить H")
    print("Q - выход")

    H = None
    h_error_mean_mm = None
    h_error_max_mm = None

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Не удалось получить кадр.")
            continue

        undistorted_full, runtime_view = preprocess_like_runtime(
            frame, camera_matrix, dist_coeffs
        )

        display = runtime_view.copy()
        gray = cv2.cvtColor(runtime_view, cv2.COLOR_BGR2GRAY)

        found, corners = find_chessboard(gray, CHESSBOARD_SIZE)
        if found:
            cv2.drawChessboardCorners(display, CHESSBOARD_SIZE, corners, found)

        lines = [
            "Phasse 2: H computing",
            "Frame uses undistortion",
            "Press 'H' to compute H",
            "Q - exit",
        ]
        draw_text_block(display, lines, color=(0, 255, 255))

        cv2.imshow("Runtime-compatible H calibration", display)
        cv2.imshow("Undistorted full", undistorted_full)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            cap.release()
            cv2.destroyAllWindows()
            return

        elif key == ord("h"):
            if not found:
                print("Шахматная доска не найдена на cropped-undistorted кадре.")
                continue

            src_pts = corners.reshape(-1, 2).astype(np.float32)
            dst_pts = objp[:, :2].astype(np.float32)

            H, mask = cv2.findHomography(src_pts, dst_pts, method=0)

            if H is None:
                print("Не удалось вычислить H.")
                continue

            predicted_mm = cv2.perspectiveTransform(
                src_pts.reshape(-1, 1, 2), H
            ).reshape(-1, 2)

            errors = np.linalg.norm(predicted_mm - dst_pts, axis=1)
            h_error_mean_mm = float(np.mean(errors))
            h_error_max_mm = float(np.max(errors))

            print("\n=== ГОМОГРАФИЯ H ===")
            print("H:\n", H)
            print("Средняя ошибка H, мм:", h_error_mean_mm)
            print("Максимальная ошибка H, мм:", h_error_max_mm)
            break

    np.savez(
        H_CALIB_FILE,
        H=H,
        crop_x1=np.array([CROP_X1], dtype=np.int32),
        crop_y1=np.array([CROP_Y1], dtype=np.int32),
        crop_x2=np.array([CROP_X2], dtype=np.int32),
        crop_y2=np.array([CROP_Y2], dtype=np.int32),
        chessboard_size=np.array(CHESSBOARD_SIZE, dtype=np.int32),
        square_size_mm=np.array([SQUARE_SIZE_MM], dtype=np.float32),
        h_error_mean_mm=np.array([h_error_mean_mm], dtype=np.float64),
        h_error_max_mm=np.array([h_error_max_mm], dtype=np.float64),
    )

    print(f"\nМатрица H сохранена в {H_CALIB_FILE}")

    # ============================================
    # ЭТАП 3. СРАЗУ ТЕСТИРОВАНИЕ
    # ============================================
    print("\nЭтап 3: тестирование H")
    testCoordinates(H, cap, camera_matrix, dist_coeffs)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
