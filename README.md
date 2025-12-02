mc_kortex
==

Interface between [kortex] and [mc-rtc].

Dependencies
------------

This package requires:
- [mc_kinova]

It can easily be installed using the superbuild extension [mc-kinova-interface-superbuild].

Usage
--

1. Install this project dependencies.
2. Install this project using `cmake` or `ninja`.
3. Configure properly your `~/.config/mc_rtc/mc_rtc.yaml` to match your needs.
4. Make sure you set your `Timestep` to 1ms as the interface does not support lower frequencies.
5. Add the following entry to your `Datastore` and to set it, to allow switch between Position control and torque control. Default: `Position`

```cpp
datastore().make<std::string>("ControlMode", "Position");
```

For this last point your configuration file typically look like this:
```yaml
MainRobot: kinova
Enabled: <your_controller>
Timestep: 0.001

# Set a LogPolicy suitable for real-time
LogPolicy: threaded

# Kortex specific configuration
Kortex:
  init_posture:
    on_startup: false
    posture: [0.0, 0.4173, 3.1292, -2.1829, 0.0, 1.0342, 1.5226]
  torque_control:
    mode: custom # If want to use modified control loop [default, feedforward, custom]
    friction_compensation:
      velocity_threshold: 0.034906585 # in rad
      acceleration_threshold: 0.5 # in rad, for cases where velocity is below threshold
      compensation_values: [2.8, 2.8, 2.8, 2.8, 1.8, 1.8, 1.8]
    integral_term:
      mu: 0.9
      gains: [10.0, 10.0, 10.0, 10.0, 300.0, 400.0, 800.0]

  kinova_default: # Name of the robot in the controller
    ip: 192.168.1.10
    username: <your_username>
    password: <your_password>
```

Run the program:

```bash
mc_kortex
```

[mc_rtc]: https://github.com/jrl-umi3218/mc_rtc
[mc_kinova]: https://github.com/mathieu-celerier/mc_kinova
