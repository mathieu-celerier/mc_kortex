# mc_kortex

Interface between **Kinova Kortex** and **mc_rtc**.

This repository provides a real-time interface that allows Kinova Gen3 robot to be controlled through the mc_rtc framework, supporting both position and torque control modes.

---

## Features

* Integration of the Kinova **Kortex API** with **mc_rtc**
* Support for **position** and custom **torque** control
* Compatible with multiple Kinova robot variants (gripper, camera, force sensors, etc.)
* Designed for **1 kHz (1 ms)** real-time control

---

## Dependencies

The following dependencies are required:

* [mc_rtc]
* [mc_kinova]

It is **strongly recommended** to install this project using the [mc-rtc-superbuild]. Manual builds using `cmake` or `ninja` are possible but not advised unless you know exactly what you are doing.

### Enabling Kinova support in the superbuild

After following the mc-rtc-superbuild installation tutorial:

```bash
cd mc-rtc-superbuild/build
ccmake .
```

Enable the option:

```
WITH_Kinova = ON
```

Then configure and build as usual.

---

## Usage

### mc_rtc configuration

Configure your `~/.config/mc_rtc/mc_rtc.yaml` file to match your setup. A complete example is provided below.

> **Important**
>
> The Kortex interface **only supports a control frequency of 1 kHz**.
> Make sure the `Timestep` is set to `0.001`.

---

## Using mc_kortex in an mc_rtc Controller

### Control mode selection

When developing a controller for a Kinova robot, you can use the mc_rtc `Datastore` to switch between **position** and **torque** control modes.

#### Default: Position control

```cpp
datastore().make<std::string>("ControlMode", "Position");
```

#### Torque control

```cpp
datastore().make<std::string>("ControlMode", "Torque");
datastore().make<std::string>("TorqueMode", "Custom");
```

### Infinite joint safety

Kinova robots have joints with infinite rotation. To prevent the robot from performing unintended full rotations, you **must** expose the posture task used by your controller:

```cpp
datastore().make_call(
  "getPostureTask",
  [this]() -> mc_tasks::PostureTaskPtr { return postureTask; }
);
```

This allows mc_kortex to properly track joint wrapping and avoid discontinuities.

---

## Example `mc_rtc.yaml`

```yaml
MainRobot: Kinova
# --- Other supported variants ---
# MainRobot: KinovaGripper
# MainRobot: KinovaCamera
# MainRobot: KinovaCameraGripper
# MainRobot: KinovaBotaDS4
# MainRobot: KinovaBota

Enabled: <your_controller>
Plugins: [<your_plugins>]

# Real-time control at 1 kHz
Timestep: 0.001

# Log policy suitable for real-time execution
LogPolicy: threaded

Kortex:
  # Settings shared by every robot of the controller
  default:
    ip: 192.168.1.10
    username: admin   # Default username (may differ on your setup)
    password: admin   # Default password

    init_posture:
      on_startup: false
      posture: [0.0, 0.4173, 3.1292, -2.1829, 0.0, 1.0342, 1.5226]

    torque_control:
      mode: custom  # Options: default, feedforward, custom

      # Gain on the Kortex API transfer function
      lambda: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]

      friction_compensation:
        velocity_threshold: 0.05        # rad/s
        acceleration_threshold: 1.0     # rad/s^2 (used when velocity is below threshold)

        # Example friction parameters — tune for your robot
        stiction: [4.0, 4.0, 4.0, 4.0, 1.5, 1.5, 1.5]
        coulomb:  [3.5, 3.5, 3.5, 3.5, 1.2, 1.2, 1.2]
        viscous:  [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0]

      # Leaky integrator, sum with the Kortex transfer function
      integral_term:
        theta: 2
        gain: 10

  # Optional: a section named after a robot overrides the defaults for that
  # robot only. Robots that use the shared settings need no section at all.
  kinova_second_arm:
    ip: 192.168.1.11
    torque_control:
      mode: feedforward
```

### Defaults and per-robot overrides

Everything under `default` applies to every actuated robot of the controller,
so the usual case of several robot variants pointing at the same arm is written
once. A section named after a robot is merged on top of the defaults,
recursively and key by key: `kinova_second_arm` above changes its address and
its torque control mode while keeping the shared credentials, friction
parameters and initial posture.

`username` and `password` fall back to `admin` when neither level sets them.
`ip` has none: a robot for which neither level provides an address is reported
as not configured and left alone. A section that matches no robot of the
controller configures nothing and is reported as such, which also catches a
misspelled robot name.

Configurations written before `default` existed, with the settings directly
under `Kortex` and the connection parameters repeated in each robot section,
are read exactly as they used to be: without a `default` section, the `Kortex`
section itself provides the defaults.

---

## Running Your Controller

Once everything is configured, start your controller with:

```bash
mc_kortex
```

---

## Hardware diagnostic

`mc_kortex --diagnostic` connects straight to the arm and prints a read-only
hardware report, without starting a controller. Use it when a joint misbehaves,
or to check the arm before a session.

It is not a general health check: it goes deep on the torque path and only skims
the rest. A joint reporting a constant torque looks identical in the Web App, in
`BaseCyclic` feedback and in the logs whether the sensor is dead, the
communication is broken or the calibration is wrong — separating those needs the
calibration block and the raw strain gauges. See
[`doc/diagnostic.md`](doc/diagnostic.md).

```bash
mc_kortex --diagnostic                        # 3s sampling window
mc_kortex --diagnostic --diagnostic-duration 10
mc_kortex --diagnostic --robot kinova_bota    # a single robot of the Kortex section
mc_kortex --diagnostic --diagnostic-dump gauges.csv  # every gauge sample to CSV
```

Connection parameters are taken from the `Kortex` section of your `mc_rtc.yaml`:
the `default` section, then every robot section that overrides it. Entries sharing
an IP address are diagnosed once, so a set of variants pointing at the same arm
produces a single report. `--robot <name>` resolves that robot's parameters even
when it has no section of its own.

For every actuator the report gives:

* identity: type, serial number, firmware version
* live feedback over the sampling window: torque, motor current, temperatures,
  and the **number of distinct torque values**, which is what separates a dead
  torque sensor from a joint that simply is not loaded
* the fault and warning banks, decoded (`STRAIN_GAUGE_MISMATCH`,
  `MAXIMUM_TORQUE`, ...)
* the torque offset and torque calibration: global gain and offset, plus the
  gain and offset of each of the four strain gauges
* the raw ADC value and the converted value of each of the four strain gauges,
  each with its own distinct-value count

Comparing the raw gauge against its converted value settles the most common
ambiguity: **live raw gauges with a converted value stuck at zero means the
sensor is healthy and its calibration is missing**, while a raw ADC stuck at a
constant means the gauge or its wiring is dead.

The strain gauges are read over the actuator cyclic service, which answers on
the TCP router in any servoing mode, so the default run reads them without
touching the arm. `--diagnostic-low-level` exists only as a fallback: if the
service does not answer, it retries inside a brief low level servoing window,
sending no command, restoring the previous servoing mode straight after
(including on `SIGINT`/`SIGTERM` or if it fails part way) and reporting any
fault raised during the window. Keep the area around the arm clear if you use
it. `--diagnostic-low-level-duration` sets the gauge sampling window, one
second by default; note that this is a sample count divided by a nominal
100 Hz, and the real rate is closer to 6 Hz, so the wall-clock window is
substantially longer than the number suggests.

Move the joints by hand during the sampling window so the torque sensors are
actually exercised.

The report ends with a per-joint verdict. Exit code is `0` when every actuator
looks healthy, `1` when an anomaly was found, `2` when the arm could not be
reached.

The diagnostic never writes a control mode or a calibration. It temporarily
changes the actuator custom data selection to read the individual strain
gauges, and — only when `--diagnostic-low-level` is given *and* the gauges did
not answer otherwise — the servoing mode; both are restored afterwards.

---

## Demos and Examples

For a complete working setup with demos and example controllers, see:

* [industry-ready-phri/mc-rtc-superbuild]

This repository provides a ready-to-run environment showcasing multiple control strategies using mc_rtc and the Kinova Gen3.

[mc_rtc]: https://github.com/jrl-umi3218/mc_rtc
[mc_kinova]: https://github.com/mathieu-celerier/mc_kinova
[Kinova Kortex API]: https://github.com/Kinovarobotics/Kinova-kortex2_Gen3_G3L
[mc-rtc-superbuild]: https://github.com/mc-rtc/mc-rtc-superbuild/
[industry-ready-phri/mc-rtc-superbuild]: https://github.com/industry-ready-phri/mc-rtc-superbuild

