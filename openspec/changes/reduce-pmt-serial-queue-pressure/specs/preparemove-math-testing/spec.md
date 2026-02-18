## ADDED Requirements

### Requirement: Queue-safe chunked serial framing
The firmware SHALL split outgoing PMT report messages into fixed-size chunks that are safe for the target serial buffer constraints before enqueueing for transmission.

#### Scenario: Long PMT line is chunked before enqueue
- **WHEN** a PMT output line exceeds the configured safe chunk payload size
- **THEN** the firmware enqueues multiple ordered chunks where each chunk length is within the configured limit

### Requirement: Low-overhead queue operations for PMT output
The firmware SHALL use a low-overhead queue strategy for PMT output that avoids expensive dynamic hot-path operations that can destabilize MCU performance.

#### Scenario: PMT output avoids heavy dynamic queue churn
- **WHEN** PMT emits its deterministic test matrix output
- **THEN** the enqueue/dequeue path avoids repeated high-cost operations equivalent to frequent dynamic `erase` and substring-copy churn in the hot path

## MODIFIED Requirements

### Requirement: Structured serial test reporting
The system SHALL emit structured serial output per test case that includes input values, computed result fields, and pass/fail status with diagnostic reason, using queue-safe chunked framing compatible with constrained serial buffers.

#### Scenario: Per-case serial report is emitted
- **WHEN** a prepareMove test case completes
- **THEN** the system prints a machine-parseable test result that preserves case identifier, input parameters, computed timing summary, and verdict fields even when output is transmitted in fixed-size chunks
