## 1. Refactor prepareMove for testable computation

- [x] 1.1 Introduce a computation result type for prepareMove math (overall timing, per-axis computed values, status/reason).
- [x] 1.2 Extract current formula path into a single canonical computation routine reused by runtime move preparation.
- [x] 1.3 Update prepareMove integration to consume computation result and preserve existing runtime behavior for valid MAP/MAJ moves.

## 2. Implement isolation-safe test execution path

- [x] 2.1 Add a dedicated command/entry path for prepareMove math tests separate from normal move commands.
- [x] 2.2 Enforce isolation guards (reject test requests while controller is moving/preparing; prevent move-start side effects during tests).
- [x] 2.3 Ensure test execution does not send synchronized start or other motion-triggering CAN actions.

## 3. Add edge-case handling and verdict logic

- [x] 3.1 Implement explicit tiny-angle and quantization classification (effective motion vs no-effective-motion) without generic failure.
- [x] 3.2 Add deterministic pass/fail criteria for synchronization-model validity per test case.
- [x] 3.3 Ensure diagnostics include clear failure reason codes for invalid/boundary inputs.

## 4. Add responsive serial reporting

- [x] 4.1 Define stable one-line serial report format for test case start/result.
- [x] 4.2 Print per-case inputs, computed output summary, and verdict fields in machine-parseable format.
- [x] 4.3 Add optional verbosity control so test output is detailed without disturbing normal runtime logs.

## 5. Verification and documentation

- [x] 5.1 Create a deterministic test matrix (nominal, mixed-distance, all-zero, tiny-angle, boundary SP/AC) for firmware-side execution.
- [ ] 5.2 Execute the matrix and confirm all scenarios produce classified, reproducible outcomes.
- [x] 5.3 Update project documentation with how to run prepareMove math tests and interpret serial output.
