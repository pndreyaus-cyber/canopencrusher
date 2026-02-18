## Why

The prepareMove function is critical for synchronized multi-axis motion because it computes per-axis velocity and acceleration/deceleration profiles. Before exhibition use, this logic needs a dedicated, repeatable test feature that validates normal and edge-case inputs (including extremely small angles) without controller instability or unexpected errors.

## What Changes

- Add a firmware-level testing feature focused on prepareMove math behavior and output validation.
- Define and execute a broad test matrix covering nominal, mixed-distance, and extreme small-angle cases.
- Add responsive serial reporting for each test case, including inputs, computed per-axis outputs, pass/fail status, and error details.
- Verify handling of tiny motion requests (for example 0.000001 degrees) so computation remains valid and does not fail unexpectedly, while respecting actuator minimum limits.
- Provide deterministic pass/fail criteria for synchronization expectations (same start/end timing model and same accel/decel timing model).

## Capabilities

### New Capabilities
- `preparemove-math-testing`: Adds an in-firmware test mode/feature to validate prepareMove calculations across many scenarios and publish structured serial test results.

### Modified Capabilities
- None.

## Impact

- Affected code: Move profile calculation path around prepareMove in MoveControllerBase, plus test orchestration/reporting entry points.
- Interfaces: Serial output format extended for test reporting; optional test command(s) may be added.
- Validation: Introduces a repeatable verification workflow for motion math correctness and synchronization assumptions.
- Risk: Additional test/reporting code may increase serial traffic and requires clear isolation from normal runtime motion control.
