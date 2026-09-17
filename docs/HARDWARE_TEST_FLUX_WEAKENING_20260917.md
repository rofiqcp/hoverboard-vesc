# Hardware Test Report — Flux Weakening Runtime

Date: 2026-09-17
Repository: `/home/otomasi/agv/hoverboard-vesc`
Branch: `v1`
Baseline before FW implementation: `78391be feat: keep current sensing active in zero-command standby`
Target MCU: STM32F103RCT6, dual FOC, USART2 921600 baud
Reference: `/home/otomasi/agv/referensi/bldc`

## Objective
Implement priority-1 runtime Flux Weakening compatible with VESC FOC behavior, then verify it on LEFT and RIGHT motors without degrading current sensing, safety, or realtime communication.

## Implementation Summary
- `foc_fw_current_max`, `foc_fw_duty_start`, `foc_fw_ramp_time`, `foc_fw_q_current_factor`, and `foc_fw_backoff` are now runtime-active.
- FW request is calculated from actual absolute duty above `foc_fw_duty_start`.
- FW current is ramped toward its duty-based target.
- FW injects negative D-axis current: `Id_target = -I_fw`.
- Q-axis compensation is applied before motor-current circle limiting.
- Backoff reduces available FW when Iq tracking error indicates inadequate torque/current headroom.
- FW is enabled only in VESC-relevant closed-loop modes and ramps back to zero when leaving the active region.
- Generic F103 canonicalization no longer silently overwrites FW configuration to disabled values.
- Safe default remains `foc_fw_current_max = 0 A`, therefore FW is disabled unless explicitly configured.

## Firmware / Build Verification
Final USART2 firmware used for hardware tests:
- Source hash: `0x5AC0C91C`
- Confirmed image CRC16: `0x8CD6`
- USART2 build: PASS
- USART3 build: PASS
- USART2 RAM: 73.2% (`36000 / 49152` bytes)
- USART2 Flash: 71.7% (`188072 / 262144` bytes)
- USART3 RAM: 73.3% (`36008 / 49152` bytes)
- USART3 Flash: 72.7% (`190484 / 262144` bytes)

Host regression:
- `VESC_REALTIME_SEMANTICS_PASS`
- `PY_VESC_DUAL_PACKET_PASS`
- `VESC_TOOL_CLI_CONSOLIDATION_PASS`
- `git diff --check`: PASS

## Test Configuration
Single-motor FW qualification used temporary RAM-only parameters:
- `foc_fw_current_max = 1.0 A`
- `foc_fw_duty_start = 0.55`
- `foc_fw_ramp_time = 0.5 s`
- `foc_fw_q_current_factor = 0.05`
- `foc_fw_backoff = 0.2`
- Speed command: `6000 ERPM`

## LEFT Motor — 6000 ERPM
Steady-state result:
- ERPM mean: `6020`
- Duty mean: `0.782`
- Active FW current: approximately `0.52 A`
- `Id_set`: approximately `-0.52 A`
- Measured Id mean: `-0.505 A`
- Measured Iq mean: `0.156 A`
- Imotor mean: `0.536 A`
- Ibat mean: `0.409 A`
- Vd mean: `-6.27 V`
- Vq mean: `17.79 V`
- Fault: `0`
- Current trip: `0`
- RX CRC error: `0`
- RX queue drop: `0`
- FW returned to `0 A` after STOP.

Conclusion: PASS. Negative Id injection follows FW command closely and the current loop remains stable.

## RIGHT Motor — 6000 ERPM
Steady-state result:
- ERPM mean: `6005`
- Duty mean: `0.753`
- Active FW current: approximately `0.46 A`
- `Id_set`: approximately `-0.46 A`
- Measured Id mean: `-0.448 A`
- Measured Iq mean: `0.183 A`
- Imotor mean: `0.489 A`
- Ibat mean: `0.337 A`
- Vd mean: `-5.78 V`
- Vq mean: `17.22 V`
- Fault: `0`
- Current trip: `0`
- RX CRC error: `0`
- RX queue drop: `0`
- FW returned to `0 A` after STOP.

Conclusion: PASS. RIGHT follows the same FW behavior as LEFT with no fault or trip.

## Dual-Motor Qualification
A conservative dual test used temporary RAM-only settings:
- `foc_fw_current_max = 0.5 A`
- `foc_fw_duty_start = 0.40`
- `foc_fw_ramp_time = 0.5 s`
- `foc_fw_q_current_factor = 0.05`
- `foc_fw_backoff = 0.2`
- Speed command: `4500 ERPM` both motors.

Observed FW state near the end of the run:
- LEFT active FW: `0.126 A`, `Id_set = -0.126 A`
- RIGHT active FW: `0.125 A`, `Id_set = -0.125 A`
- LEFT measured Id mean: `-0.094 A`
- RIGHT measured Id mean: `-0.095 A`
- LEFT duty mean: `0.568`
- RIGHT duty mean: `0.565`
- LEFT Imotor mean: `0.356 A`
- RIGHT Imotor mean: `0.292 A`
- LEFT Ibat mean: `0.243 A`
- RIGHT Ibat mean: `0.235 A`
- Fault LEFT/RIGHT: `0 / 0`
- Current trip LEFT/RIGHT: `0 / 0`
- RX CRC errors: `0`
- RX queue drops: `0`

Conclusion: PASS for simultaneous FW activation and current regulation on both motors. The LEFT speed loop showed more ERPM variation than RIGHT during this unloaded dual test; this is treated as a speed-loop/load observation, not a FW functional failure.

## Communication Regression
With FW configuration restored to default/non-active state, VESC Tool style traffic was re-tested for 5 seconds:
- RT LEFT: `49.97 Hz`, `250/250` replies
- RT RIGHT: `49.95 Hz`, `250/250` replies
- PPM App Data: `19.98 Hz`, `100/100`
- ADC App Data: `20.00 Hz`, `100/100`
- CHUK App Data: `20.00 Hz`, `100/100`
- RX drop LEFT/RIGHT: `0 / 0`
- CRC errors LEFT/RIGHT: `0 / 0`
- Result: `VESC_RT50_APP20_PASS`

During aggressive dual 6000-ERPM speed-loop tests, short GET_VALUES polling deadlines could time out while process-gap increased. This did not create motor fault/trip or CRC/drop. Therefore 6000-ERPM dual speed-loop/telemetry stress is not claimed as fully qualified by this report.

## Final Safety State
After testing, temporary FW parameters were restored in RAM to the safe/non-active profile:
- `foc_fw_current_max = 0 A`
- `foc_fw_duty_start = 0.8`
- `foc_fw_ramp_time = 0 s`
- `foc_fw_q_current_factor = 0.05`
- `foc_fw_backoff = 2.0`
- Active FW current LEFT/RIGHT: `0 / 0 A`
- Id setpoint from FW LEFT/RIGHT: `0 / 0 A`
- Fault LEFT/RIGHT: `0 / 0`
- Current trip LEFT/RIGHT: `0 / 0`

## Overall Result
Priority-1 Flux Weakening runtime implementation is functionally verified on LEFT and RIGHT independently and simultaneously at conservative dual-load conditions. The key VESC behavior is present: duty-triggered FW request, ramped negative Id injection, Q-current sharing, current-circle limiting, backoff, and decay to zero outside the active region. Default FW remains disabled until explicitly configured.

Remaining follow-up is performance qualification of the dual 6000-ERPM speed-loop plus aggressive realtime polling, which is separate from the demonstrated correctness of the FW current-control path.
