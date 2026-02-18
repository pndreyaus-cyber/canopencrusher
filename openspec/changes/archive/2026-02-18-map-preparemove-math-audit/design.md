## Context

`prepareMove` in `MoveControllerBase` computes synchronized per-axis profile velocity and acceleration from target positions, requested speed, and requested acceleration. This function is safety-critical for MAP/MAJ because all axes must follow a common movement-time model.

Current behavior mixes calculation and controller state mutation, has edge-case ambiguity (very small distances, zero-like values, all-zero movement), and does not expose structured intermediate values needed for robust testing.

The required test feature must be isolated from normal runtime control and must provide responsive serial output of inputs, computed outputs, and pass/fail results.

## Goals / Non-Goals

**Goals:**
- Add an isolated in-firmware test pathway for `prepareMove` math verification that does not interfere with normal runtime motion control.
- Allow `prepareMove` internals/signature to evolve to return richer computed data needed by tests.
- Validate synchronization math across nominal and extreme cases, including very small angles (for example `0.000001` deg) without unexpected logic errors.
- Print structured, responsive serial test output per case (inputs, computed per-axis values, verdict).

**Non-Goals:**
- Full load/performance stress testing over long command sequences (handled by separate MAP load-test change).
- CAN bus timing validation or motor hardware characterization.
- Redesign of the command protocol beyond minimal test command/reporting additions.

## Decisions

### 1) Split math computation from motion-side effects

**Decision:** Introduce a pure calculation result model (e.g., `PrepareMoveComputationResult`) that contains:
- overall profile type/timings (accel, cruise, total)
- per-axis target steps, relative steps, computed velocity/acceleration
- status code + diagnostic reason

`prepareMove` will use this result to apply values to axes for runtime moves; test mode will call the same computation path without triggering move execution.

**Rationale:** Enables deterministic testing and richer diagnostics without duplicating formula logic.

**Alternatives considered:**
- Keep existing `bool prepareMove(...)` and inspect mutable axis state in tests. Rejected: poor observability and brittle coupling.
- Duplicate formulas in a separate test function. Rejected: high drift risk.

### 2) Explicit isolation boundary for test feature

**Decision:** Add a dedicated test command path that:
- runs only in explicit test mode
- does not call MAJ movement start sequence
- does not send RPDO/SYNC
- does not mutate runtime move status unless intentionally running a controlled dry-run context

If controller is currently moving, test requests are rejected with a clear status.

**Rationale:** Prevents test traffic/logic from changing behavior of live motion control.

**Alternatives considered:**
- Reuse MAP command with hidden flags. Rejected: high accidental-use risk in normal runtime.

### 3) Structured serial output contract for responsiveness

**Decision:** Emit one-line machine-parseable records for each test case (start/result), including:
- case id, command parameters, profile type
- per-axis computed velocity/acceleration and relative movement
- timing values and pass/fail with failure reason

Output remains human-readable but stable enough for future parser automation.

**Rationale:** Fast operator feedback at exhibition-prep stage and repeatable evidence of correctness.

**Alternatives considered:**
- Verbose free-text logs only. Rejected: hard to compare and automate.

### 4) Edge-case math policy (including tiny angles)

**Decision:** Define explicit acceptance behavior:
- tiny non-zero angle requests are valid inputs;
- computation must return a valid profile/result status instead of generic error/crash;
- if conversion/quantization leads to zero-step effective movement, report as a non-fatal classified outcome (e.g., `NO_EFFECTIVE_MOTION`) rather than logic failure;
- enforce minimum actuator command constraints in result generation (documented clamping/rounding behavior).

**Rationale:** Tiny inputs are expected in real use; behavior must be deterministic and explainable.

**Alternatives considered:**
- Reject all sub-threshold angles as invalid params. Rejected: less useful for calibration and precision exploration.

## Risks / Trade-offs

- **[Risk] Result model expansion increases code complexity** → **Mitigation:** keep one canonical computation function and minimal enums/structs.
- **[Risk] Serial output overhead affects loop timing if overused** → **Mitigation:** test mode only; concise line format; optional verbosity level.
- **[Risk] Quantization/clamping may hide physical limitations** → **Mitigation:** include explicit fields showing pre-quantized and final commanded values.
- **[Risk] Test mode accidentally invoked during runtime operation** → **Mitigation:** explicit mode gate + reject when any axis is in moving/prepared states.

## Migration Plan

1. Introduce computation result types and refactor `prepareMove` internals behind them.
2. Keep runtime MAP/MAJ behavior functionally equivalent for valid existing cases.
3. Add test command handler and serial reporting path, guarded by runtime isolation checks.
4. Validate with deterministic firmware test scenarios and compare against expected outputs.
5. Rollback strategy: disable test command path and revert to previous `prepareMove` behavior if regression is detected.

## Open Questions

- What exact command name should trigger the prepareMove math tests (new command vs debug-only build command)?
- Should tiny-angle outcomes that quantize to zero steps be marked PASS with classification or WARN requiring operator acknowledgment?
- What final serial record schema should be frozen now to stay compatible with the later Python load-test change?
