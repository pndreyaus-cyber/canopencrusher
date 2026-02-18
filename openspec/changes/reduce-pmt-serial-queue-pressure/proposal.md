## Why

The PMT path is unstable on the target because serial output must be sent in small fixed-size chunks (64-byte buffer limit), while the current queue path performs heavy dynamic-string operations (`std::vector<String>`, `push_back`, `erase`, `substring`) that are costly on the microcontroller. This must be fixed to keep PMT reliable while preserving queue-based output safety.

## What Changes

- Keep queue-first output architecture (`addDataToOutQueue`) as the only PMT output entry point.
- Enforce fixed-size output chunking so every emitted unit fits the target serial buffer constraints.
- Replace expensive dynamic queue operations in the hot path with a lightweight queue representation suitable for STM32 constraints.
- Preserve PMT summary/verdict semantics while reducing allocation/copy overhead in serial message handling.
- Ensure the solution is reusable for future test commands with similar serial-output pressure.

## Capabilities

### New Capabilities
- None.

### Modified Capabilities
- `preparemove-math-testing`: Update reporting requirements to include fixed-size chunked output and low-overhead queue handling that remains safe under constrained MCU resources.

## Impact

- Affected code: PMT test runner/reporting path plus serial queue internals around `addDataToOutQueue`/`sendData`.
- Interfaces: PMT line framing may change due to chunking, but command-level behavior remains compatible.
- Runtime behavior: reduced heap churn and lower risk of queue-related stalls or overflow on MCU.
- Risk: queue implementation change has broader impact than PMT-only throttling; mitigated by keeping public output API stable.
