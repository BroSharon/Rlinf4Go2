# Rlinf4Go2

This project is one of the patches for integrating Unitree Go2 into the Rlinf framework. Its main purpose is to wrap the process of collecting Go2 motion trajectories within Rlinf, such that remote controller commands are routed through Rlinf before being sent to Go2.

[中文使用说明](docs/source-zh/rst_source/examples/embodied/go2.rst)

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
