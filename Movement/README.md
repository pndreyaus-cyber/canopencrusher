# Automated MAP move command executor for CANCrusher

This script is used to execute multiple move commands one by one
Currently the move command parameters are passed as a txt file

## Parameters
### Required
* `--port` — What COM port to use. Example: `--port=COM7`
* `--input-file` — The location of the file containing move instructions
* `--axes` — Anount of axes the file contains

### Optional
* `--baud` — Serial baud rate. Default is 115200
* `--position-tolerance` — Accepted position error. In steps. Default is 10
* `--move-timeout` — Maximum allowable time between the MAP send and the reception of the MAP status. In seconds. Default is 10
* `--poll-ms` — Polling period of the RPP command. In milliseconds. Default is 500

## Setup

```powershell
cd move_test_app
python -m venv .venv # do this only once
.\.venv\Scripts\Activate.ps1 
pip install -r requirements.txt
```

## Run

```powershell
.\.venv\Scripts\activate
python run_move_tests.py --port=COM7 --input_file=paths/path_1_6_axes.txt
```