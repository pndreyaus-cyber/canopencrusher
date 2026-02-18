## Context

The target serial path is constrained by a 64-byte buffer, so output must be transmitted in bounded chunks. PMT diagnostics can produce long lines, and the current queue path uses expensive dynamic operations (`std::vector<String>`, `push_back`, `erase`, `substring`) that amplify CPU and memory pressure on STM32.

This change must preserve queue-based serial safety (single output funnel), enforce fixed-size framing compatible with the 64-byte constraint, and reduce dynamic allocation/copy overhead. It also must preserve deterministic PMT verdict visibility so operators can trust test outcomes.

## Goals / Non-Goals

**Goals:**
- Keep PMT output on `addDataToOutQueue` and avoid direct unsynchronized `Serial2.println` in PMT path.
- Ensure every queued output unit is split into fixed-size chunks that safely fit the target serial buffer constraints.
- Replace heavy dynamic queue operations in the hot path with a lightweight, deterministic queue mechanism.
- Preserve PMT core verdict visibility (`RESULT`/`SUMMARY`) while minimizing runtime overhead.

**Non-Goals:**
- Changes to core prepareMove math formulas or motion semantics.
- Introducing external dependencies or transport protocols.

## Decisions

### 1) Enforce fixed-size chunk framing at queue ingress

**Decision:** `addDataToOutQueue` shall normalize each logical message into fixed-size chunks sized for the hardware limit (payload plus terminator compatibility), then enqueue chunks only.

This guarantees that no individual serialized fragment exceeds the target buffer constraints during output.

**Why:** The critical constraint is per-fragment size, not only line count.

**Alternatives considered:**
- Keep long messages and rely on downstream truncation (rejected: unsafe and non-deterministic).
- Direct `Serial2.println` per fragment (rejected: breaks queue safety model).

### 2) Replace String-vector queue hot path with lightweight structure

**Decision:** Replace `std::vector<String>` + frequent `erase/substr` queue operations in the hot path with a lightweight queue representation (for example fixed-size ring buffer of char chunks or equivalent low-allocation structure).

Maintain the same public enqueue/dequeue API behavior so call sites remain stable.

**Why:** Root cause includes excessive dynamic memory/copy operations, not just PMT verbosity.

**Alternatives considered:**
- PMT-only message-throttling with unchanged queue internals (rejected: mitigates symptoms, not root cause).

### 3) Keep PMT contract stable, optimize internals

**Decision:** Keep PMT command interface and essential line semantics (`START`/`RESULT`/`SUMMARY`) while allowing internal chunking/framing changes.

If needed, verbose detail remains configurable, but optimization priority is queue-efficiency first.

**Why:** Avoids protocol churn while enabling safe performance improvements.

**Alternatives considered:**
- Redefine PMT output protocol completely (rejected: unnecessary compatibility risk).

### 4) Reusable pattern for future test commands

**Decision:** Implement chunking + lightweight queue as shared infrastructure so future test/report features reuse the same constrained-safe output path.

**Why:** Prevents repeated fixes for each new diagnostic feature.

**Alternatives considered:**
- PMT-specific optimization only (rejected: poor scalability).

### 5) Deterministic behavior and observability

**Decision:** Output behavior remains deterministic under identical inputs, and summary-level diagnostics remain available to confirm successful test completion.

**Why:** Maintains operator trust and debuggability after infrastructure changes.

## Risks / Trade-offs

- **[Risk] Queue refactor affects all serial output users** → **Mitigation:** preserve `addDataToOutQueue` contract and validate core command responses.
- **[Risk] Fixed-size chunking may split human-readable lines awkwardly** → **Mitigation:** keep clear delimiters/terminator policy and document parsing expectations.
- **[Risk] Memory footprint of fixed buffers** → **Mitigation:** tune ring size conservatively for STM32 RAM budget and expose compile-time constants.

## Migration Plan

1. Define fixed chunk-size constants aligned with the 64-byte output constraint.
2. Refactor `addDataToOutQueue` to frame/split messages into bounded chunks before queue insertion.
3. Replace dynamic hot-path queue operations with lightweight queue storage/consumption strategy.
4. Validate PMT and regular command output compatibility (correct ordering, delimiters, completion lines).
5. Rollback strategy: preserve previous queue implementation behind compile-time switch or staged commit fallback.

## Open Questions

- What exact chunk payload size should be used (for example 63 bytes payload + terminator) for safest compatibility?
- Should the lightweight queue be implemented as a fixed-size ring of char arrays or as an equivalent static-buffer design?
- Do you want output chunking to be global for all commands immediately, or enabled first for PMT path and then expanded?
