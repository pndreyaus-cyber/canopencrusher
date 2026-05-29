"""
Алгоритм
----
  основной цикл:
    считать кадр -> распознать кубики -> посчитать скользящее среднее
    if среднее > 1:
      - остановить конвейере
      - считать свежий кадр после остановки конвейера -> распознать кубики
      - для каждого кубики: запустить сбор роботом
      - по окончании сбора выкинуть старые кадры
      - запустить конвейер
"""

import argparse
import configparser
import glob
import time
from collections import deque

import cv2
import serial

from camera_gui.vision_centroid_lib.detector import WhiteObjectCentroidDetector
from Movement.CoordinateTransform import CoordinateTransform
from Movement.Point import Point

# ── tuneable constants ────────────────────────────────────────────────────────
CUBE_HEIGHT     = 28        # mm
CONVEYOR_ON     = "POW OP1 WV0"
CONVEYOR_OFF    = "POW OP1 WV1"
SETTLE_SEC      = 1.0       # пауза после остановки конвейера перед захватом кадра
FLUSH_FRAMES    = 5         # кадры для отброса, чтобы захваченный кадр был свежим
POST_PICK_FLUSH = 20        # кадры для отброса после всего цикла захвата
SERIAL_TIMEOUT  = 5         # таймаут блокировки pyserial readline (с)
RECONNECT_PAUSE = 1.0       # пауза между попытками переподключения
PICK_THRESHOLD = 0.95


# ── serial port helpers ───────────────────────────────────────────────────────

def _find_port() -> str | None:
    """Вернуть первый /dev/ttyACM* из доступных или None."""
    ports = sorted(glob.glob("/dev/ttyACM*"))
    return ports[0] if ports else None


def _open_port(baud: int) -> serial.Serial:
    """Попытка подключения к порту с автоматическими повторными попытками при неудаче."""
    while True:
        port = _find_port()
        if port:
            try:
                ser = serial.Serial(
                    port, baud,
                    timeout=SERIAL_TIMEOUT,
                    dsrdtr=False, 
                    rtscts=False,
                )
                print(f"[serial] подключен: {port}")
                return ser
            except serial.SerialException as exc:
                print(f"[serial] не удалось открыть {port}: {exc}")
        else:
            print("[serial] не найдено /dev/ttyACM*, повторная попытка…")
        time.sleep(RECONNECT_PAUSE)


def _check_rms_reply(reply: str | None) -> bool:
    reply_strip = reply.strip() if reply else ""
    return reply_strip == "RMS OK JA2 JB2 JC2 JD2"


# ── SerialConn – transparent reconnect wrapper ────────────────────────────────

class SerialConn:
    """
    Каждый вызов send_and_wait() автоматически повторяется после отключения USB
    """

    def __init__(self, baud: int) -> None:
        self.baud = baud
        self._ser = _open_port(baud)

    # ------------------------------------------------------------------
    def _reconnect(self) -> None:
        """Закрыть порт, дождаться его появления, открыть заново и отправить команду RMS."""
        print("[serial] Потеряна связь с USB. Переподключение...")
        try:
            self._ser.close()
        except Exception:
            pass

        while True:
            self._ser = _open_port(self.baud)

            try:
                self._ser.write(b"RMS\n")
                self._ser.flush()
                print("[serial] >> RMS")
            except serial.SerialException:
                print("[serial] Не получилось отправить команду RMS")
                continue

            reply, disc = self._read_until(prefix="RMS", timeout_s=2.0)
            if disc:
                print("[serial] Отключился во время команды RMS")
                continue
            if _check_rms_reply(reply):
                print("[serial] RMS OK – восстанавливаю работу")
                return
            print(f"[serial] RMS не принят ({reply}), повторная попытка…")
            time.sleep(RECONNECT_PAUSE)

    # ------------------------------------------------------------------
    def _read_until(self, prefix: str, timeout_s: float) -> tuple[str | None, bool]:
        """
        Читает строки, пока не встретит строку, начинающуюся с *prefix*, или не истечет таймаут.
        Возвращает (line, disconnected).
        disconnected=True означает, что порт отключился; вызывающий код должен переподключиться.
        """
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                raw = self._ser.readline()
            except (serial.SerialException, OSError, TypeError):
                return None, True
            if not raw:
                continue
            line = raw.decode("utf-8", errors="ignore").strip()
            if line:
                print(f"[serial] << {line}")
            if line.startswith(prefix):
                return line, False
        return None, False  # timeout (not a disconnect)

    # ------------------------------------------------------------------
    def send_and_wait(
        self,
        command: str,
        prefix: str,
        timeout_s: float = 15.0,
    ) -> str | None:
        """
        Если команда не может быть отправлена из-за отключения USB, переподключается и повторяет попытку.
        Возвращает ответ, начинающийся с *prefix*, или None при таймауте (не при отключении).
        """
        while True:
            print(f"[serial] >> {command}")
            try:
                self._ser.write(f"{command}\n".encode("ascii", errors="ignore"))
                self._ser.flush()
            except (serial.SerialException, OSError):
                self._reconnect()
                continue  # повторная отправка после переподключения

            reply, disconnected = self._read_until(prefix, timeout_s)
            if disconnected:
                self._reconnect()
                # Робот мог отправить ответ, пока переподключались. Этот ответ надо выкинуть
                try:
                    self._ser.reset_input_buffer()
                except Exception:
                    pass
                continue  # повторная отправка после переподключения

            if reply is None:
                print(f"[serial] таймаут ожижания команды '{prefix}' (command: {command})")
            return reply

    # ------------------------------------------------------------------
    def close(self) -> None:
        try:
            self._ser.close()
        except Exception:
            pass


# ── camera helpers ────────────────────────────────────────────────────────────

def _flush_and_read(
    cap: cv2.VideoCapture,
    settle_sec: float,
    flush: int,
) -> "cv2.Mat | None":
    """Спит, отбрасывает *flush* кадров, возвращает последний (или None при ошибке)."""
    if settle_sec > 0:
        time.sleep(settle_sec)
    frame = None
    for _ in range(flush):
        ok, frame = cap.read()
        if not ok:
            return None
    return frame


# Сконфигурировать команду движения

def _mac(x: float, y: float, z: float, sp: float = 10.0, ac: float = 2.0) -> str:
    return f"MAC PX{x:.3f} PY{y:.3f} PZ{z:.3f} OR0 OP0 OW0 SP{sp:.1f} AC{ac:.1f}"


# ── main loop ─────────────────────────────────────────────────────────────────

def run(cfg: configparser.ConfigParser) -> None:
    # paths
    model_path            = cfg["paths"]["model_path"]
    camera_calib_path     = cfg["paths"]["camera_calib_path"]
    crop_path             = cfg["paths"]["crop_path"]
    transform_matrix_path = cfg["paths"]["transform_matrix_path"]

    # camera
    confidence  = cfg.getfloat("camera", "confidence_level")
    camera_id   = cfg.getint("camera",  "camera_id")

    # robot / serial
    baud          = cfg.getint("robot", "baud")
    move_conveyor = cfg.getboolean("robot", "move_conveyor")

    port_cfg = cfg["robot"]["port"]
    po_timeout = cfg.getint("move", "po_timeout")
    move_reply_timeout = cfg.getint("move", "move_reply_timeout")

    move_speed = cfg.getfloat("move", "move_speed")
    move_acceleration = cfg.getfloat("move", "move_acceleration")
    up_down_speed = cfg.getfloat("move", "up_down_speed")
    up_down_acceleration = cfg.getfloat("move", "up_down_acceleration")

    # move parameters
    robot_y_low     = cfg.getint("move", "robot_y_low")
    robot_y_high    = cfg.getint("move", "robot_y_high")
    window_width    = cfg.getint("move", "window_width")
    cube_hover_dist = cfg.getint("move", "cube_hover_dist")
    cube_pick_delta = cfg.getint("move", "cube_pick_delta")
    drop_x          = cfg.getint("move", "drop_x")
    drop_y          = cfg.getint("move", "drop_y")
    drop_z          = cfg.getint("move", "drop_z")

    # ── initialise subsystems ─────────────────────────────────────────────────
    transform = CoordinateTransform()
    transform.load_transform_matrix_from_file(transform_matrix_path)

    detector = WhiteObjectCentroidDetector(
        model_path=model_path,
        camera_calibration_path=camera_calib_path,
        conf_threshold=confidence,
        crop=crop_path,
    )

    conn = SerialConn(baud)

    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        raise RuntimeError("Камера недоступна")

    cube_cnt_q: deque[int] = deque(maxlen=window_width)

    if move_conveyor:
        conn.send_and_wait(CONVEYOR_ON, "POW", po_timeout)

    # ── main loop ─────────────────────────────────────────────────────────────
    try:
        while True:
            # ШАГ 1. Чтение кадра с камеры
            ok, frame = cap.read()
            if not ok:
                print("Камера отключилась. Выход...")
                break

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

            # ШАГ 2. Проверка криетрия наличия кубиков на кадре
            count = 0
            if detector.has_objects(frame):
                for centroid in detector.get_centroids_px(frame):
                    if robot_y_low <= centroid[0] <= robot_y_high:
                        count += 1

            cube_cnt_q.append(count)

            avg = sum(cube_cnt_q) / len(cube_cnt_q)

            if avg < PICK_THRESHOLD:
                continue

            # ШАГ 3. Кубики есть. Остановка конвейера
            if move_conveyor:
                conn.send_and_wait(CONVEYOR_OFF, "POW", po_timeout)

            # ШАГ 4. Берем свежий кадр после остановки
            fresh = _flush_and_read(cap, SETTLE_SEC, FLUSH_FRAMES)
            if fresh is None:
                print("Не смог сделать кадр после остановки конвейера")
                if move_conveyor:
                    conn.send_and_wait(CONVEYOR_ON, "POW", po_timeout)
                cube_cnt_q.clear()
                continue

            # ШАГ 5. Распознавание кубиков на кадре
            result = detector.process_frame(fresh, annotate=True)
            #cv2.imshow("Annotated", result.annotated_frame)

            # ШАГ 6. Расчет координат кубов в С.К. робота
            valid: list[Point] = []
            for centroid in result.centroids_px:
                if robot_y_low <= centroid[0] <= robot_y_high:
                    pt       = Point(centroid[0], centroid[1], CUBE_HEIGHT)
                    robot_pt = transform.homography_theoretical_to_robot_coordinates(pt, 240)
                    valid.append(robot_pt)

            if not valid:
                print("Кубы не распознаны после остановки конвейера")
                if move_conveyor:
                    conn.send_and_wait(CONVEYOR_ON, "POW", po_timeout)
                cube_cnt_q.clear()
                continue

            # ШАГ 7. Сбор кубиков
            for rp in valid:
                px, py, pz = rp.x, rp.y, rp.z

                # Шаг 7.1. Подъехать над кубиком
                conn.send_and_wait(_mac(px, py, pz + cube_hover_dist, move_speed, move_acceleration), "MAC", move_reply_timeout)

                # Шаг 7.2. Опуститься к кубику
                conn.send_and_wait(_mac(px, py, pz - cube_pick_delta, up_down_speed, up_down_acceleration), "MAC", move_reply_timeout)

                # Шаг 7.3. Включить присоску
                conn.send_and_wait("GRW 1", "GRW", po_timeout)

                # Шаг 7.4. Подняться с кубиком
                conn.send_and_wait(_mac(px, py, pz + cube_hover_dist, up_down_speed, up_down_acceleration), "MAC", move_reply_timeout)

                # Шаг 7.5. Подъехать к точке сброса 
                conn.send_and_wait(_mac(drop_x, drop_y, drop_z, move_speed, move_acceleration), "MAC", move_reply_timeout)

                # Шаг 7.6. Выключить присоску
                conn.send_and_wait("GRW 0", "GRW", po_timeout)

            # ШАГ 8. Пропустить старые кадры
            for _ in range(POST_PICK_FLUSH):
                cap.read()

            # ШАГ 9. Включить конвейер
            if move_conveyor:
                conn.send_and_wait(CONVEYOR_ON, "POW", po_timeout)

            cube_cnt_q.clear()

    finally:
        cap.release()
        cv2.destroyAllWindows()
        conn.close()


# ── entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config.ini")
    args = ap.parse_args()

    cfg = configparser.ConfigParser()
    cfg.read(args.config)

    run(cfg)
