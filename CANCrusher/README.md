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

## Serial Output Strategy
Serial output is currently line-oriented and based on an internal `String` buffer:
* Logical messages are assembled as complete text lines in memory.
* Each line is sent using `Serial2.println(...)`, which appends the newline terminator.
* If the size of the queue extends 100 messages, the head of the queue is forced to be send to Serial
