# Rlinf4Go2
This project is one of the patches for integrating Unitree Go2 into the Rlinf framework. Its main purpose is to wrap the process of collecting Go2 motion trajectories within Rlinf, such that remote controller commands are routed through Rlinf before being sent to Go2.

[中文使用说明](examples/embodiment/go2_sport/README.zh-CN.md)

## Run and Review Go2 Recording

Use the [Go2 guide](examples/embodiment/go2_sport/README.md) to check state reception, inspect joystick input, and record a trajectory. RLinf saves the velocity command and the states before and after each control step. The JSON report keeps SDK velocity feedback with host reception times.

| Location | Purpose |
| --- | --- |
| [examples/embodiment/go2_sport](examples/embodiment/go2_sport) | Command-line entry point and usage |
| [rlinf/envs/real/go2](rlinf/envs/real/go2) | Drive environment and state/action spaces |
| [Unitree mobility adapter](rlinf/robotics/parts/mobility/unitree_go2.py) | SDK2 state reception and velocity commands |
| [Robotics tests](tests/unit_tests/test_robotics.py) | CPU checks with a mock Unitree SDK |

The recording change is [commit 772cd4f](https://github.com/BroSharon/Rlinf4Go2/commit/772cd4f6692d49c75e4f0b4d8c77fac0be50f92e). Follow [RLinf #1948](https://github.com/RLinf/RLinf/pull/1948) for the upstream Go2 hardware interface. Hardware state reception and mock recording have been verified. Active handset recording still needs an on-site test.
