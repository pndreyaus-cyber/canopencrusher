# 6-axis robot-manipulator control system using stm32 and Arduino

## Features
* Motors are controlled via CANopen commands
* Synchronised movement (motors start and stop at the same time)
* PI-controller setting
* Zero initialization of encoders
* Built-in prepareMove math self-test mode (`PMT`) with structured serial output

## Serial Commands (core)

* `MAJ...` - absolute move in degrees
* `MAP...` - absolute move with speed/acceleration in percent of configured maximums
* `MRJ...` - relative move in degrees
* `RPP...` - request axis position(s) in steps
* `RMS` - request motor/controller status
* `ZEI...` - zero-initialize selected axis/all axes
* `PMT` - run deterministic firmware-side prepareMove math tests (terse output)
* `PMTV1` - run the same tests with verbose per-axis output
* `PMTV0` - run tests with terse output and disable verbose mode

## PMT Output Format

`PMT` emits machine-parseable single-line records:

* `PMT START ...` - test case input snapshot
* `PMT RESULT ...` - computed timings, profile class, quantization flags, pass/fail verdict
* `PMT AXIS ...` - per-axis computed details (only when verbose mode is enabled)
* `PMT SUMMARY ...` - total/passed/failed counters

## Serial Framing and Queue Strategy

Serial output now uses bounded chunk framing through `addDataToOutQueue`:

* Each queued chunk payload is capped at `63` bytes (`SERIAL_OUT_CHUNK_PAYLOAD_MAX`).
* A newline chunk is appended after each logical message.
* `sendData` transmits one queued chunk at a time using `Serial2.write(...)`.

Queue internals are implemented as a fixed-size ring buffer (`SERIAL_OUT_QUEUE_CAPACITY`), which avoids front-erase and substring-heavy queue churn.

## Reuse for Future Test Commands

Any future diagnostics command should write output only via `addDataToOutQueue` to automatically inherit chunk framing and lightweight queue behavior.

## Rollback Note

If field behavior regresses, rollback by restoring the previous queue implementation around `addDataToOutQueue`/`sendData` from version control, then retest PMT output and core command replies.
