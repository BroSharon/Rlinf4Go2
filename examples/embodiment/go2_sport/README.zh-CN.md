# Go2 六方向真机控制

[English](README.md)

通过 RLinf 控制 Go2 前进、后退、左移、右移、左转和右转。宇树原厂控制器负责步态与关节控制。

2026-11-16 的六方向运动和停稳已获现场操作者确认，见[历史验收归档](https://github.com/QuincyQAQ/RLinf/blob/a253e5e657299103fa5e766c24b0f05eb5fb854f/artifacts/go2_sport_20261116/README.zh-CN.md)。本次迁移保留驱动行为，并检查当前 RLinf 接口兼容性；迁移版本已通过真机只读检查，轨迹录制的运动验证仍待完成。

## 代码位置

目录遵循仓库 [AGENTS.md](../../../AGENTS.md) 和[新增机器人说明](../../../docs/source-zh/rst_source/extending/new_robot.rst)。

| 位置 | 内容 |
|---|---|
| [rlinf/robotics/parts/mobility/unitree_go2.py](../../../rlinf/robotics/parts/mobility/unitree_go2.py) | 原厂 SDK2 状态订阅、速度指令、超时停止请求 |
| [rlinf/robotics/robots/unitree_go2.py](../../../rlinf/robotics/robots/unitree_go2.py) | 机器人组合与硬件配置 |
| [rlinf/envs/real/go2/](../../../rlinf/envs/real/go2/) | 注册环境 `Go2DriveEnv-v1` |
| [run.py](run.py) | 只读检查与单方向手动测试入口 |
| [历史验收归档](https://github.com/QuincyQAQ/RLinf/tree/a253e5e657299103fa5e766c24b0f05eb5fb854f/artifacts/go2_sport_20261116) | 原始 JSON、脚本快照与 SHA-256 清单 |
| [tests/unit_tests/test_robotics.py](../../../tests/unit_tests/test_robotics.py) | 驱动、六方向及退出路径的 SDK 替身测试 |

## 使用

按[安装与运行说明](../../../docs/source-zh/rst_source/examples/embodied/go2.rst)准备依赖。在仓库根目录预览单个方向，`<interface>` 替换为连接 Go2 的网卡名称：

```bash
python -m examples.embodiment.go2_sport.run --interface <interface> --direction forward --dry-run
```

预览不连接硬件。六个方向名为 `forward`、`backward`、`left`、`right`、`turn_left`、`turn_right`。实际运动前须在现场核实站立状态、场地和遥控器，再移除 `--dry-run` 并输入 `GO`。单次运行只测一个方向，默认指令持续时间为 1 秒；`--seconds 2` 可改为 2 秒。日志写入 `outputs/go2_sport/`。

### 录制状态轨迹

只读检查通过后，每次运动指定一个尚不存在的目录。将 `eno1` 换成连接机器人的网卡，在现场按提示确认运动。

```bash
python -m examples.embodiment.go2_sport.run --interface eno1 --direction forward --record-dir outputs/go2_episodes/forward_001
```

完成一次 1 秒运动后，`forward_001/pickle_episode/` 下会生成一个文件。10 条动作都是下发的 `[vx, vy, yaw_rate]`，单位依次为 m/s、m/s、rad/s。`observations[t]` 和 `observations[t+1]` 分别是第 `t` 条指令前后的状态。共 11 帧，每帧包含姿态、角速度、12 个关节的位置与速度、电池电压和四足原始受力值。最后一步标记为 `truncated`。文件名中的 `_fail.pkl` 表示这项任务没有提供成功标签。SDK 估计的运动速度及接收时间另存于 JSON 报告的 `speeds` 字段。

### 用 Go2 手柄录制示范

启用 `--handset` 后，RLinf 每个控制步读取 `rt/wirelesscontroller` 最近一次摇杆输入，按 `[0.3 * ly, -0.3 * lx, -0.5 * rx]` 生成三维速度，通过 Go2 环境下发。各轴的死区为 0.05，右摇杆纵轴不参与速度计算。JSON 报告逐步保存摇杆值和主机接收时间，pickle 保存下发的速度指令。

首次使用前，请按当前固件支持的方式隔离原厂摇杆直控，并在支撑架上确认两件事：拨动摇杆时能读到方向正确的轴值，同时机器人保持静止；现场独立于这条手柄输入链路和主机的停止装置有效。脚本把控制设置留给现场操作者。如果 RLinf 启动前拨杆就能让狗行走，需要先完成这项联调。可在支撑架上运行以下只读命令，在 JSON 的 `steps[].mapped_velocity` 中核对拨杆方向。此模式只接收数据。

```bash
python -m examples.embodiment.go2_sport.run --interface eno1 --inspect-handset --seconds 2
```

预览录制命令：

```bash
python -m examples.embodiment.go2_sport.run --interface eno1 --handset --seconds 2 --record-dir outputs/go2_episodes/handset_001 --dry-run
```

现场核实后去掉 `--dry-run`，双摇杆回中，输入 `GO`。第一步检查摇杆回中；采集中收到按键、轴值无效或手柄输入超过 0.2 秒未更新，脚本会结束本轮并请求停止。完整运行两秒会保存 20 条动作和 21 帧状态。中断的回合只留 JSON 诊断，不导出半段轨迹。

在仓库根目录运行下列命令可用 CPU 验证两种输入。测试为每种输入录制两个回合，读回文件逐步核对模拟 SDK 收到的指令。

```bash
PYTHONPATH=.:tests:tests/unit_tests python -m pytest -q tests/unit_tests/test_robotics.py -k go2_sport_recording_round_trip
```

## 与 ABC 运控的关系

本目录发送三维机身速度指令，不加载训练权重。[go2_abc](../go2_abc/MOTION.zh-CN.md) 则加载 ABC policy，输出 12 关节动作并换算电机力矩；其真机验证单独记录。

## Limitations

六方向归档属于现场功能确认，距离、转角和停止距离的量化验收尚未完成。SDK 速度估计与停止请求均不能代替现场观察；进程退出、网络断开等情况下仍需独立停止手段。环境奖励为零，尚未提供真机强化学习任务。
