# `mc_kortex --diagnostic` — notes

Usage and flags: [README](../README.md#hardware-diagnostic). This file covers what is not obvious from the code.

## Why it exists

A case where a joint was constantly giving `0.000` as a measurement occurred and needed to be diagnosed.
A joint reporting a constant `0.000` N·m has three causes with different remedies:

| Cause | Remedy |
|---|---|
| Gauge or wiring dead | actuator replacement |
| Actuator↔base comms fault | service |
| Torque calibration erased | vendor supplies coefficients, write them back |

The Web App, `BaseCyclic` feedback and `.bin` logs show the same zero in all three cases. Separating them requires the calibration block and the raw strain-gauge values, i.e. reading below the reported torque.

## Scope

- Deep on the torque path: calibration coefficients, individual gauges, raw and converted values, distinct-value counts.
- Context only: current, voltage, temperatures, fault banks.
- Not covered: encoders, brakes, gripper, interconnect, latency, joint limits, kinematic calibration.

`ReadTorqueCalibration` and `WriteTorqueCalibration` are internal-use-only, and factory coefficients for a given serial exist only at Kinova.

## What it reads

| Source | Service | Gives |
|---|---|---|
| Live feedback | `BaseCyclic::RefreshFeedback` | torque, current, voltage, temperatures, distinct-value counts |
| Fault/warning banks | `BaseCyclic` feedback | e.g. `STRAIN_GAUGE_MISMATCH` |
| Torque calibration | `ActuatorConfig::ReadTorqueCalibration` | global gain/offset, per-gauge gains/offsets |
| Strain gauges | `ActuatorCyclic` custom data | raw ADC and converted value per gauge |
| Arm identity | `DeviceConfig`, device id 0 | base serial, model, part number, firmware, MAC |

## Discriminators

Conversion applied by the actuator:

```
torque = global_gain × mean_g( raw_g × gain_g + offset_g ) + global_offset
```

**Distinct-value count.** A live sensor carries noise, so a healthy joint reports a different value in nearly every sample. One distinct value over a whole window means it is not reporting, regardless of load.

**Raw vs converted**, same message:

| Observation | Conclusion |
|---|---|
| raw live, converted stuck at zero | sensor healthy, calibration missing |
| raw stuck at a constant | gauge or wiring dead |

Both work on a stationary arm.

## Kortex behaviours

- `ActuatorCyclic` (service 11) returns `UNSUPPORTED_SERVICE` on the UDP real-time router; it answers on the **TCP router**. This is not a servoing-mode restriction — low-level servoing does not help. `sampleStrainGauges` probes both routers before changing any selection.
- The transport blocks for minutes on an unreachable arm, so `isReachable()` pre-checks with a non-blocking socket and a 2 s `select()`.
- `GetCalibrationResult` is unsupported (service 9, function 35) on *every* actuator at firmware `0x03020501`. Probed once on the first actuator, the rest marked `UNAVAILABLE`, never counted as an anomaly.
- `DeviceConfig::CalibrationItem` has no torque entry (`COGGING`, `MAGNETIC`, `MOTOR`, `POSITION_RANGE` only): no self-calibration exists for a torque sensor.
- Fault banks are decoded through the protobuf descriptor (`SafetyIdentifierBankA_IsValid` / `_Name`), not a hand-written table.

## Sampling rate

Gauge sampling runs at ~6 Hz, not 100 Hz: each sample costs seven sequential TCP round trips.

- `--diagnostic-low-level-duration` is a sample count divided by a nominal 100 Hz, so wall-clock duration is much longer — a 25 s request ran 115 s.
- Gauges of different actuators in one row are read ~175 ms apart. Irrelevant for static checks, dominant error when correlating fast motion across joints.

## Writes and recovery

Read-only except for two items, both restored:

1. **Custom data selection**, to expose the gauge channels. Previous selection read first, written back at the end.
2. **Servoing mode**, only with `--diagnostic-low-level` *and* only if the gauges did not answer otherwise. Does not trigger in practice.

`LowLevelServoingGuard` restores the previous mode from its destructor and from `SIGINT`/`SIGTERM` handlers, using atomic globals so the handler does not allocate or lock. A `SIGTERM` skips destructors and previously left the arm in `LOW_LEVEL_SERVOING`. `--diagnostic-reset-servoing` is the manual recovery path.

## Open items

- `checkActuatorsFaultBanks` is commented out at `KinovaRobot.cpp:515` (since `7d41191`, 2024-06-25), so actuator faults never reach the logs. Cannot be re-enabled as-is: it calls `error_and_throw` on any non-zero bit, blocking startup on a faulted arm. Wanted: log decoded faults once, add the banks as mc_rtc log entries.
- `getActuatorFaultList` at `KinovaRobot.cpp:999` uses a hand-written 27-bit table missing `BRAKE_DRIVE_FAULT` (`0x8000000`) and `BRAKE_RELEASE_MOTION_OUT_OF_RANGE` (`0x10000000`). Should decode through the descriptor.
- No write path for calibration. A `--write-torque-calibration` mode would make restoring vendor-supplied coefficients a one-liner; not built, it would be the first path that writes to the robot.

## Originating case

Third row of the table above: all ten calibration coefficients zeroed, gauges live and load-responsive. Settled on a stationary arm by torque feedback with one distinct value over 300 samples (σ = 0, healthy joints 298–300) and raw gauges changing in 19–20 of 20 samples while converted values held `0.0000`.

Full investigation recorded outside this repository — it concerns one arm, not the tool.
