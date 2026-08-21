# vision-centroid-lib

Мини-библиотека для:
- детекции объектов через Ultralytics YOLO,
- поиска белого контура внутри найденного bbox,
- возврата координат центроида в миллиметрах.

## Что дает библиотека

Главные пользовательские варианты использования:

- функциональный API: `configure_detector(...)`, `has_objects(frame)`, `get_centroids_mm(frame)`
- объектный API: `WhiteObjectCentroidDetector(...).has_objects(frame)` и `WhiteObjectCentroidDetector(...).get_centroids_mm(frame)`

Дополнительно есть `process_frame(frame)`, если нужен кадр с аннотациями.

## Структура

```text
vision_centroid_lib/
├── vision_centroid_lib/
│   ├── __init__.py
│   └── detector.py
├── example_camera.py
├── requirements.txt
├── requirements-headless.txt
├── pyproject.toml
├── Dockerfile
└── README.md
```

## Установка без Docker

```bash
python -m venv .venv
```

### Windows

```bash
.venv\Scripts\activate
pip install -U pip
pip install -r requirements.txt
pip install -e .
```

### Linux / macOS

```bash
source .venv/bin/activate
pip install -U pip
pip install -r requirements.txt
pip install -e .
```

## Использование в своем коде

```python
import cv2
from vision_centroid_lib import configure_detector, get_centroids_mm, has_objects

configure_detector(
    model_path="best.pt",
    camera_calibration_path="camera_calibration.npz",
    homography_path="calibration_4pt_homography.npz",
    conf_threshold=0.5,
    crop=(40, 430, 0, 640),
)

cap = cv2.VideoCapture(0)
ret, frame = cap.read()
if not ret:
    raise RuntimeError("Не удалось получить кадр")

flag = has_objects(frame)
centroids = get_centroids_mm(frame)

print(int(flag))
print(centroids)
```

## Поведение функций

### 1. Проверка наличия объектов

```python
flag = detector.has_objects(frame)
print(int(flag))
```

- `0` — ничего не найдено
- `1` — найден хотя бы один объект с вычисленным центроидом

### 2. Получение координат центроидов

```python
centroids = detector.get_centroids_mm(frame)
```

Пример результата:

```python
[(123.45, 87.11), (200.02, 54.88)]
```

Если объектов нет, вернется пустой список:

```python
[]
```

## Важное замечание

В библиотеке уже добавлен простой кэш последнего кадра. Поэтому если ты подряд вызываешь `has_objects(frame)` и `get_centroids_mm(frame)` для одного и того же объекта `frame`, повторный запуск YOLO для этого же кадра не выполняется.

Если нужна максимально явная и контролируемая обработка, используй:

```python
result = detector.process_frame(frame)
flag = result.found_objects
centroids = result.centroids_mm
```

Так обработка кадра выполняется один раз.

## Docker

Docker-режим удобен для воспроизводимой среды, но для работы с обычной веб-камерой и `cv2.imshow()` на другом ПК часто проще использовать обычный `venv` + `requirements.txt`.

Если нужен headless-запуск без GUI, можно собрать контейнер:

```bash
docker build -t vision-centroid-lib .
```

Запуск:

```bash
docker run --rm -it vision-centroid-lib
```

Для серверного режима без окон в контейнере используется `opencv-python-headless`.
