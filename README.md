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

  # Robot connection parameters
  kinova:
    ip: 192.168.1.10
    username: admin   # Default username (may differ on your setup)
    password: admin   # Default password

  kinova_bota:
    ip: 192.168.1.10
    username: admin
    password: admin

  kinova_bota_ds4:
    ip: 192.168.1.10
    username: admin
    password: admin

  kinova_camera:
    ip: 192.168.1.10
    username: admin
    password: admin

  kinova_camera_gripper:
    ip: 192.168.1.10
    username: admin
    password: admin
```

---

## Running Your Controller

Once everything is configured, start your controller with:

```bash
mc_kortex
```

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

