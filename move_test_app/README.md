# MAP move command automated tester

Standalone Python application for automated testing of `MAP` command + `RPP` verification over Serial.

This tool is independent from the firmware code and can run from this folder only.

## What it tests

- Axes: `1, 3, 4, 5` (default)
- Positions (deg): `+1, -1, +5, -5, +17, -17, +37, -37, +51, -51, +5, -5`
- Velocity (%): `1, 5, 10, 15, 20, 30, 40, 50, 60, 70, 80, 90`
- Acceleration (%): `1, 5, 10, 15, 20, 30, 40, 50, 60, 70, 80, 90`

For one axis: `12 * 12 * 12 = 1728` tests.

## Measured per test

- `target_position`
- `target_velocity`
- `target_acceleration`
- `result_position` (from `RPP`)
- `result_move_time_ms` (time until target reached within tolerance)

Additional columns included:

- `axis`, `reached_target`, `status`, `error`, `timestamp_utc`

## Setup

```powershell
cd move_test_app
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

## Run

```powershell
python run_move_tests.py --port COM7
```

### Useful options

```powershell
# Custom tolerance and timeout
python run_move_tests.py --port COM7 --position-tolerance 0.5 --move-timeout 10

# Single axis only
python run_move_tests.py --port COM7 --axes 1

# Custom output file
python run_move_tests.py --port COM7 --output results\axis1_test.xlsx
```

## Notes on protocol format

- Sent `MAP`: `MAPJA+1SP10AC20`
- Sent `RPP`: `RPPJA`
- Expected `RPP` reply format: `RPP OK JA<position>`

Axis mapping used by the script:

- Axis `1 -> A`
- Axis `2 -> B`
- Axis `3 -> C`
- Axis `4 -> D`
- Axis `5 -> E`

## Stop safely

Press `Ctrl+C` to stop. Partial results are saved to the output file.
