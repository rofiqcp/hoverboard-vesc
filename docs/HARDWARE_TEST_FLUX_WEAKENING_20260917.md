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

## Addendum: Dual 6000 ERPM + Realtime Stress Qualification
The earlier note that dual 6000-ERPM telemetry was not fully qualified is superseded by this addendum. The bottleneck was traced to dual closed-loop Hall/held-SVPWM work being executed for both motors on every 16-kHz ADC frame, not to the FW algorithm or UART bandwidth.

Runtime scheduling was optimized while preserving the control contracts:
- ADC/DC-link protection remains 16 kHz.
- LEFT current PI remains PWM/6 = 2.667 kHz on slot 0.
- RIGHT current PI remains PWM/6 = 2.667 kHz on slot 1.
- Closed-loop Hall phase + held-SVPWM updates are staggered at 8 kHz per motor.
- Hall period/rate math keeps the original 16-kHz timebase through elapsed-PWM-tick compensation.
- Full Hall edge/history processing runs only on a debounced Hall-state transition; unchanged sectors use the same interpolation/rate-limit stage directly.
- OPENLOOP/commissioning paths retain full-frame service.

After optimization, dual 6000-ERPM speed control with FW enabled (`1.0 A`, duty start `0.55`, ramp `0.5 s`, q factor `0.05`, backoff `0.2`) was tested simultaneously with VESC Tool-style RT traffic for 8 seconds:
- LEFT GET_VALUES: `400/400`, `50.01 Hz`
- RIGHT GET_VALUES: `400/400`, `49.99 Hz`
- PPM: `160/160`, `20.00 Hz`
- ADC: `160/160`, `20.01 Hz`
- CHUK: `160/160`, `20.01 Hz`
- Parser/validation errors: `0`
- RX queue drop: `0`
- TX queue drop: `0`
- TX start failure: `0`
- Fault LEFT/RIGHT: `0 / 0`
- Current trip LEFT/RIGHT: `0 / 0`
- Maximum protocol process gap: `5 ms`

Steady high-speed samples at the end of the stress test remained physically coherent:
- LEFT: `ERPM 6153`, duty `0.778`, Imotor `0.46 A`, Ibat `0.44 A`, Id `-0.42 A`, Iq `0.16 A`, fault `0`.
- RIGHT: `ERPM 6153`, duty `0.750`, Imotor `0.47 A`, Ibat `0.28 A`, Id `-0.45 A`, Iq `0.13 A`, fault `0`.

Compared with the clean pre-optimization stress run, protocol responsiveness improved from `process_gap_max_ms = 135 ms` with `4/40` GET_VALUES poll failures to `5 ms` with zero missing replies in the full 50/20-Hz stress test.

Profiler caveat: peak current-control slots still exceed the nominal 4000-cycle 16-kHz frame budget (`slot0 ≈ 4948`, `slot1 ≈ 5599` maximum in this run), so `deadline_miss` remains non-zero. The staggered scheduler makes the other slots approximately `3130..3220` cycles, providing sufficient CPU recovery margin for deterministic communications. This report therefore qualifies the observed dual-6000 control and VESC realtime workload, but does not claim zero ISR deadline misses.

Final result for this addendum: `DUAL_6000_FW_RT50_APP20_PASS`.

## Stage 3 Qualification — Q-current factor, backoff, and current-off delay
Stage 3 closes the remaining VESC `foc_run_fw()` parity around FW transitions. Firmware under test: USART2 921600 baud, candidate CRC16 `0x1196`. The implementation adds the VESC-style 1000 ms current-off delay and makes normal speed-zero enter sensing standby rather than hard release; electrical faults, E-stop, watchdog, reboot, and bootloader still bypass the delay and hard-release immediately.

### Q-current factor
LEFT was tested at 6000 ERPM with `foc_fw_current_max=1.0 A`, `foc_fw_duty_start=0.55`, `foc_fw_ramp_time=0.2 s`, and backoff disabled. With `foc_fw_q_current_factor=0`, mean active FW was `527.4 mA` and mean `Iq_target - Iq_set` was exactly `0 mA`.

With `foc_fw_q_current_factor=0.5`, mean active FW was `516.4 mA`. Mean `Iq_target - Iq_set` became `258.5 mA`; the expected value `0.5 × I_fw` was `258.2 mA`. This verifies the VESC equation `Iq_set = Iq_target - SIGN(mod_q) × I_fw × q_factor` within integer quantization.

RIGHT was independently checked with `q_factor=0.5`: mean active FW `462.6 mA`; measured magnitude `|Iq_target-Iq_set| = 231.9 mA`; expected `0.5 × I_fw = 231.3 mA`. The sign reversed correctly with RIGHT direction/modulation inversion.

### FW backoff
To create a controlled positive Iq error, LEFT was stabilized near 6000 ERPM with FW active and then changed to a small reverse-current request while the rotor remained forward. At nearly equal duty (`~0.57`), `foc_fw_backoff=0` produced active FW around `320 mA`; `foc_fw_backoff=2.0` reduced active FW to about `257 mA`. The result verifies that positive `SIGN(speed) × (Iq-Iq_target)` reduces the available FW current instead of allowing D-axis voltage demand to run away.
### Current-off delay
After steady 6000 ERPM operation, LEFT was commanded to zero speed while `COMM_ALIVE` remained active. The FW duty region initially refreshed `off_ms=1000`. Once duty dropped below the FW threshold, the delay counted down monotonically: `1000 → 909 → 815 → 722 → 629 → 536 → 443 → 350 → 257 → 164 → 71 → 0 ms`. It reached zero at approximately `1.12 s` from the zero-speed transition. During this interval the current regulator remained active, rotor speed and `Id/Iq/Imotor/Ibat` stayed observable, and no fault occurred.

RIGHT independently showed the same result: `off_ms` maximum `1000 ms`, reaching zero at approximately `1.12 s`, with fault `0`. The delay therefore exists per motor, not as a shared/global timer.

### Dual-motor + realtime stress
Both motors were then tested together at 6000 ERPM using temporary RAM-only FW settings `1.0 A / 0.55 duty / 0.5 s ramp / q=0.05 / backoff=0.2`. Simultaneously the VESC Tool traffic pattern was applied: motor realtime values at 50 Hz per motor and App Data PPM/ADC/CHUK at 20 Hz.

Results: LEFT `400/400` replies at `49.99 Hz`; RIGHT `400/400` replies at `49.99 Hz`; PPM/ADC/CHUK each `160/160` at `20.00 Hz`. Parser errors `0`, RX queue drops `0`, TX queue drops `0`, fault LEFT/RIGHT `0/0`, and current trips `0/0`. End-of-run measured values were approximately LEFT `duty=0.777, Id=-0.49 A, Iq=+0.22 A`, RIGHT `duty=0.758, Id=-0.44 A, Iq=+0.17 A`.

At roughly 0.3 s after the dual zero-speed command, FW current had already decayed to zero while the post-FW delay remained active at `683 ms` LEFT and `663 ms` RIGHT. This is consistent with the independent ~1 s countdown tests and confirms the delay survives a dual-motor transition under full telemetry load.

### Stage 3 result
PASS. Q-current sharing matches the upstream VESC equation, FW backoff reduces requested negative-D current under positive Iq tracking error, and the one-second current-off delay now preserves active zero-current modulation after leaving the FW region. Normal zero-speed uses sensing standby; hard safety releases remain immediate. Temporary FW test parameters were restored to the safe defaults (`current_max=0 A`, `duty_start=0.8`, `ramp=0`, `q_factor=0.05`, `backoff=2.0`) after testing.
### Hard-release safety check
A separate LEFT test used `FW max=0.5 A`, `duty_start=0.4`, `ramp=0.2 s`, `q=0.05`, `backoff=0.2` at approximately 4444 ERPM. Immediately before the safety command, active FW was about `162 mA`, measured `Id=-0.17 A`, `Iq=+0.23 A`, duty about `0.61`, and `off_ms=1000`.

`stop hard` was then issued. Within approximately 80 ms the motor reported `CONTROL_MODE_NONE / MC_STATE_OFF`, active FW `0`, `Id_set=0`, duty `0`, and `off_ms=0`; fault/trip remained `0/0`. This verifies that FW current-off delay never blocks the fail-safe hard-release path.