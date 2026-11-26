# Rlinf4Go2

This project is one of the patches for integrating Unitree Go2 into the Rlinf framework. Its main purpose is to wrap the process of collecting Go2 motion trajectories within Rlinf, such that remote controller commands are routed through Rlinf before being sent to Go2.

[中文使用说明](docs/source-zh/rst_source/examples/embodied/go2.rst)

## Branch Overview

This is the current **PR1** for recording Go2 handset demonstrations through the standard RLinf collector. Use `collect_data_go2` to save the body velocity commands sent to the robot and the states before and after them.

- [Early handset recording prototype](https://github.com/BroSharon/Rlinf4Go2/tree/feat/go2-handset-recording)
- [PR2 velocity tracking evaluation](https://github.com/BroSharon/Rlinf4Go2/tree/feat/go2-velocity-tracking)
- [Sim-to-real policy replay prototype](https://github.com/BroSharon/Rlinf4Go2/tree/feat/go2-handset)

## Run Go2 Collection

Follow the [Go2 guide](docs/source-en/rst_source/examples/embodied/go2.rst) to install SDK2, check state reception, and prepare the handset. Preview the collection configuration before connecting to the robot.

```bash
bash examples/embodiment/collect_data.sh collect_data_go2 --cfg job
```

| Location | Purpose |
| --- | --- |
| [Collection configuration](examples/embodiment/config/collect_data_go2.yaml) | Select Go2 handset recording through the standard collector |
| [Handset adapter](rlinf/robotics/parts/teleop/go2_handset.py) | Convert stick positions to body velocity commands |
| [Go2 environment](rlinf/envs/real/go2) | Dispatch commands and return robot state |
| [Robotics tests](tests/unit_tests/test_robotics.py) | Check the saved commands and states with a mock SDK |

RLinf saves the executed velocity commands and observations as pickle episodes. Each step also stores SDK velocity feedback and handset samples with their host reception times. The Go2 hardware integration comes from [RLinf #1948](https://github.com/RLinf/RLinf/pull/1948). Active handset recording still needs an on-site test.
