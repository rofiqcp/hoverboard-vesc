# Hardware Test — VESC Input/Battery Current Limit + Flux Weakening

Date: 2026-09-17  
Target: STM32F103RCT6 dual FOC, USART2 921600  
Reference: `/home/otomasi/agv/referensi/bldc`

## Objective

Stage 4 validates the VESC input-current limiting path when field weakening produces negative `Id`.
The fast loop already limits `Iq` with the VESC approximation `Ibus ~= mod_q * Iq`, but upstream VESC also applies a slow measured-input-current map because `Id` contributes to input power and is not represented by that approximation.

This implementation uses the real board DC-link sensors:
- LEFT: DCL
- RIGHT: DCR

The public VESC battery-current polarity is `Ibat = -ibus_counts`; the measured map therefore uses the same polarity before filtering.

## Implemented control path

`DCL/DCR -> Ibat polarity normalization -> l_in_current_map_filter LPF -> l_in_current_map_start mapping -> dynamic positive motor-current ceiling -> current circle -> Id/Iq PI`.

The existing fast VESC limiter `lo_in_current / mod_q` remains active in parallel. Regen remains on the upstream model-based `l_in_current_min / mod_q` path; the measured mapping is intentionally applied only to positive battery draw, matching upstream `update_override_limits()`.## Baseline before measured mapping

Test condition on LEFT:
- `l_in_current_max = 0.30 A`
- FW current max `1.0 A`
- FW duty start `0.55`
- FW ramp `0.2 s`
- speed target `6000 ERPM`

Stage-3 firmware, model limiter only:
- Ibat mean: `0.327 A`
- Ibat max: `0.420 A`
- Imotor mean: `0.501 A`
- Id mean: `-0.468 A`
- Iq mean: `0.157 A`
- duty mean: `0.762`
- ERPM mean: `5823`
- fault/trip: `0/0`

This demonstrates the upstream TODO directly: negative Id causes real battery current to exceed what the `mod_q * Iq` estimate alone predicts.## LEFT qualification after measured mapping

Identical test parameters, final sign-corrected firmware:
- Ibat mean: `0.289 A`
- Ibat p95: `0.320 A`
- Ibat max: `0.360 A`
- Imotor mean: `0.296 A`
- Id mean: `-0.148 A`
- Iq mean: `0.233 A`
- duty mean: `0.620`
- ERPM mean: `4617`
- fault/trip: `0/0`

Measured map evidence at steady state:
- DCL filtered Ibat: about `0.298 A`
- configured Ibat max: `0.300 A`
- base motor-current ceiling: `9.67 A`
- mapped motor-current ceiling: about `0.40 A`, reaching `cc_min_current` near the limit

The controller correctly sacrifices speed instead of violating the battery-current authority.## RIGHT qualification

With the same `0.30 A` input-current limit and FW parameters:
- Ibat mean: `0.263 A`
- Ibat p95/max: `0.280 A`
- Imotor mean: `0.501 A`
- Id mean: `-0.476 A`
- Iq mean: `0.147 A`
- duty mean: `0.765`
- ERPM mean: `6060`
- fault/trip: `0/0`

DCR filtered Ibat remained about `0.267 A`, below the 90% map threshold (`0.270 A`), so the measured map correctly stayed mostly inactive while the fast model limiter alone held battery current below the configured maximum.

## Regen observation

A separate brake test used `l_in_current_min = -0.20 A` from about 4000 ERPM.
- LEFT negative-Ibat mean: `-0.160 A`, minimum transient `-0.280 A`
- RIGHT negative-Ibat mean: `-0.034 A`, minimum transient `-0.360 A`
- fault/trip: `0/0`

This is documented as an observation, not a Stage-4 acceptance failure: upstream VESC applies the measured `m_i_in_filter` mapping only to positive battery draw. Regen remains limited by the fast `lo_in_current_min / mod_q` path and can show brief transition overshoot.## Dual-motor + realtime stress

Both motors were configured with `l_in_current_max = 0.30 A`, FW enabled, and 6000 ERPM targets while VESC Tool-style realtime polling ran.

Final motor snapshots:
- LEFT: Ibat `0.28 A`, Imotor `0.31 A`, Id `-0.21 A`, Iq `0.21 A`, duty `0.619`, ERPM `4705`
- RIGHT: Ibat `0.22 A`, Imotor `0.49 A`, Id `-0.46 A`, Iq `0.15 A`, duty `0.774`, ERPM `5925`
- fault/trip LEFT/RIGHT: `0/0`, `0/0`
- process gap max: `6 ms`

Motor RT replies were `400/400` LEFT and `400/400` RIGHT at about 50 Hz with zero CRC/drop. One run contained one stale extra PPM reply (`161/160`) after configuration changes; a clean reboot regression produced the exact expected counts:
- motor RT: `250/250` LEFT, `250/250` RIGHT
- PPM/ADC/CHUK: `100/100` each
- rates: ~50 Hz motor, ~20 Hz App Data
- RX drop: `0`
- CRC error: `0`
- process gap: `1 ms`
- result: `VESC_RT50_APP20_PASS`

## Result

Stage 4 PASS for the upstream VESC measured positive-input-current mapping with real DCL/DCR sensing and negative Id / flux weakening. Compile defaults remain `l_in_current_max=15 A`, `l_in_current_min=-15 A`, map start `0.90`, map filter `0.002`, with FW disabled unless explicitly configured. The actual EEPROM configuration observed after a clean reboot on this hardware is more conservative: input-current limits `+8/-8 A`; FW remains disabled (`foc_fw_current_max=0`) with duty start `1.0`. Stage-4 qualification parameters were RAM-only and were not persisted.