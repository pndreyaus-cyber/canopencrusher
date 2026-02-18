## ADDED Requirements

### Requirement: Deterministic prepareMove computation result
The firmware SHALL provide a deterministic computation result for each prepareMove test case, including overall timing values and per-axis computed motion parameters needed to validate synchronization math.

#### Scenario: Computation result is available for a valid multi-axis case
- **WHEN** a prepareMove test case is executed with valid axis targets, speed, and acceleration
- **THEN** the system returns a computation result containing per-axis target/relative movement and computed velocity/acceleration plus overall timing values

### Requirement: Isolated test execution from runtime motion control
The prepareMove testing feature SHALL be isolated from normal runtime motion control and SHALL NOT trigger real motion execution side effects.

#### Scenario: Test case does not start physical move sequence
- **WHEN** an operator runs a prepareMove math test
- **THEN** the system does not send movement-start side effects (including synchronized start trigger) and does not transition axes into active movement states for that test run

#### Scenario: Test request is rejected while runtime move is active
- **WHEN** a prepareMove test is requested while any axis is in a moving or move-preparation runtime state
- **THEN** the system rejects the test request with an explicit status indicating controller-busy isolation protection

### Requirement: Tiny-angle and quantization-safe behavior
The prepareMove testing feature SHALL handle extremely small non-zero angle inputs without generic logic failure, and SHALL classify quantization outcomes explicitly.

#### Scenario: Tiny non-zero angle input is processed safely
- **WHEN** a test case includes an input such as 0.000001 degrees on one or more axes
- **THEN** the system returns a classified result (for example effective-motion or no-effective-motion) instead of an unclassified error or crash

### Requirement: Structured serial test reporting
The system SHALL emit structured serial output per test case that includes input values, computed result fields, and pass/fail status with diagnostic reason.

#### Scenario: Per-case serial report is emitted
- **WHEN** a prepareMove test case completes
- **THEN** the system prints a machine-parseable one-line test result containing case identifier, input parameters, computed timing summary, and verdict fields

### Requirement: Test verdict criteria for synchronization model
The testing feature SHALL evaluate each case against explicit synchronization-model criteria and produce a pass/fail verdict.

#### Scenario: Synchronization criteria are evaluated
- **WHEN** a test case computation completes
- **THEN** the system evaluates whether the produced per-axis profile values conform to a common acceleration/deceleration timing model and reports pass/fail with reason
