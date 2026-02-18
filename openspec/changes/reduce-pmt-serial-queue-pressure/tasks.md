## 1. Define constrained serial output contract

- [x] 1.1 Add serial output framing constants for chunk payload and chunk terminator policy aligned to the 64-byte target constraint.
- [x] 1.2 Document and enforce the exact chunk boundary rule in `addDataToOutQueue` (no queued chunk may exceed configured payload size).

## 2. Refactor queue internals to lightweight operations

- [x] 2.1 Replace hot-path `std::vector<String>` queue mutation pattern (`push_back` + front `erase`) with a lightweight deterministic queue mechanism suitable for STM32.
- [x] 2.2 Ensure enqueue and dequeue paths avoid repeated expensive substring/copy churn for normal PMT and non-PMT command traffic.
- [x] 2.3 Preserve existing `addDataToOutQueue` and `sendData` call-site contract so command handlers require minimal or no interface changes.

## 3. Apply chunked framing to PMT and general output path

- [x] 3.1 Update `addDataToOutQueue` to split long logical messages into ordered fixed-size chunks before queue insertion.
- [ ] 3.2 Verify PMT output semantics (`START`, `RESULT`, `SUMMARY`) are preserved after chunking.
- [ ] 3.3 Confirm command responses for non-PMT commands (`MAJ`, `MAP`, `RPP`, `RMS`, `ZEI`) remain correctly framed and parseable.

## 4. Validate behavior and resource impact

- [ ] 4.1 Run PMT in concise and verbose modes and confirm no chunk exceeds configured payload size.
- [ ] 4.2 Verify deterministic PMT completion visibility (`SUMMARY` appears and contains correct totals) under high output volume.
- [ ] 4.3 Perform a regression check for queue stability (no obvious overflow/stall symptoms) during mixed command usage.

## 5. Documentation and rollback readiness

- [x] 5.1 Update project docs with final framing format and queue behavior expectations for tooling/parsers.
- [x] 5.2 Add implementation notes on how to reuse the chunking/queue strategy for future diagnostic test commands.
- [x] 5.3 Document rollback strategy (how to temporarily restore prior queue behavior if field issues are observed).
