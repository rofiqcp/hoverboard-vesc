# Stage 5 — VESC Speed Source Parity

Status: **SOURCE COMPLETE / NOT YET BUILT, UPLOADED, OR HARDWARE-TESTED**.

Reference: `/home/otomasi/agv/referensi/bldc` (VESC FOC implementation).
Target: STM32F103 dual-FOC firmware in this repository.

## Runtime parity

- `S_PID_SPEED_SRC_PLL`, `FAST`, and `FASTER` remain three distinct runtime modes.
- FAST and FASTER are updated from corrected electrical phase delta at the real current-control cadence.
- Phase delta is clamped to +/-60 electrical degrees as in upstream VESC.
- FAST uses LPF alpha 0.01; FASTER uses LPF alpha 0.20.
- PLL, FAST, and FASTER are updated in parallel regardless of the speed-PID selection.
- PLL includes upstream-style wind-up protection: `|PLL| <= 3 * |FAST|`.
- PLL/FAST/FASTER continue updating from commanded electrical phase during OPENLOOP/OPENLOOP_PHASE commissioning, so public PLL RPM remains available to Detect-All as in upstream VESC.
- Estimators are seeded from live sensor speed on initialization/config changes to avoid artificial zero-speed steps.
- `s_pid_speed_source` selects **speed PID feedback only**.
- Public FOC RPM / `COMM_GET_VALUES` remains PLL-based, independent of speed-PID source.
- Both scaled and float GET_VALUES paths use the same PLL ERPM authority; no Hall/ABI fallback is exposed as public FOC RPM.
- Direction normalization remains in `mc_interface`/packet layer, matching VESC `DIR_MULT` semantics.
- Field-weakening backoff and D/Q decoupling use FAST speed, matching upstream VESC.

## Motor-configuration parity

- Compile/default speed PID source is PLL, matching upstream VESC.
- Invalid speed-source values canonicalize to PLL.
- Older EEPROM revisions without a persisted speed-source field migrate to PLL.
- EEPROM mode flags retain 2 bits, so value `2 = FASTER` is preserved.
- Terminal accepts `speed_src 0PLL|1FAST|2FASTER`.
- VESC MC configuration wire now serializes/deserializes `s_pid_speed_source` at the upstream position.
- The surrounding FOC block was realigned to upstream order, including HFI ambiguity fields, `foc_hfi_max_err`, `foc_hfi_reset_erpm`, `foc_fw_backoff`, zero-duty/overmod fields, and `foc_sl_erpm_start`.
- FOC control/current sample-mode bytes are serialized as enum bytes, not collapsed booleans.
- Unsupported HFI/short-low-side functionality is canonicalized to deterministic disabled values rather than falsely advertised as active.

## Project-specific exception

Hall position remains the project-specific cascade `Position -> Speed -> Iq -> Current` because Hall angular resolution is too coarse for the direct stock-VESC position-to-Iq path. Therefore this custom Hall-position cascade reuses the configured speed feedback source. Encoder position remains the direct position PID path.

## Pending qualification

No build, upload, regression execution, or hardware run is claimed in this document. The next qualification phase must cover MC-config wire roundtrip, EEPROM FASTER persistence, old-EEPROM migration to PLL, hot source switching at nonzero ERPM, Detect-All/open-loop public RPM, LEFT/RIGHT at 3000 and 6000 ERPM, dual 6000 ERPM, public-PLL vs PID-source separation, and RT50/App20 under PLL/FAST/FASTER.

## Public/API semantics

- `mcpwm_foc_get_erpm_motor()` now returns PLL ERPM only, matching upstream `mcpwm_foc_get_rpm()`; it does not fall back to Hall/ABI for public FOC RPM.
- `mc_interface_get_rpm()` applies direction normalization on top of that PLL value, matching upstream `DIR_MULT` placement.
- Dedicated FAST and FASTER getters remain available for diagnostics without changing public RPM authority.
- `speed_est` diagnostics explicitly distinguish raw internal speed, PLL/FAST/FASTER internal estimates, selected PID feedback, and direction-normalized VESC public ERPM.
- Raw Hall/ABI speed is retained only for estimator seeding, readiness/safety logic, and diagnostics—not as public FOC RPM authority.

Prepared test sources are intentionally not executed yet. They include MC-config wire roundtrip of nontrivial enum values, FASTER persistence, old-EEPROM migration to PLL, and static assertions that public RPM remains PLL while the speed PID selector changes independently.
