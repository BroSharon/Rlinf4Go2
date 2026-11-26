# Go2 Six-Direction Hardware Control

[中文](README.zh-CN.md)

Use RLinf to move the Go2 forward, backward, sideways left or right, and turn left or right. The Unitree controller handles gait and joint control.

The operator confirmed motion and stopping in all six directions on 2026-11-16; see the [historical acceptance archive](https://github.com/QuincyQAQ/RLinf/blob/a253e5e657299103fa5e766c24b0f05eb5fb854f/artifacts/go2_sport_20261116/README.md). This migration preserves the driver behavior and checks compatibility with this RLinf revision. The migrated version has passed a hardware state reception check. Motion recording still needs an on-site test.

## Code Locations

The layout follows [AGENTS.md](../../../AGENTS.md) and [Adding a Robot](../../../docs/source-en/rst_source/extending/new_robot.rst).

| Location | Purpose |
|---|---|
| [rlinf/robotics/parts/mobility/unitree_go2.py](../../../rlinf/robotics/parts/mobility/unitree_go2.py) | SDK2 state reception, velocity commands, and timeout stop requests |
| [rlinf/robotics/robots/unitree_go2.py](../../../rlinf/robotics/robots/unitree_go2.py) | Robot composition and hardware configuration |
| [rlinf/envs/real/go2/](../../../rlinf/envs/real/go2/) | Registered `Go2DriveEnv-v1` environment |
| [run.py](run.py) | Read-only checks and manual single-direction tests |
| [Historical acceptance archive](https://github.com/QuincyQAQ/RLinf/tree/a253e5e657299103fa5e766c24b0f05eb5fb854f/artifacts/go2_sport_20261116) | Original JSON reports, script snapshots, and SHA-256 manifest |
| [tests/unit_tests/test_robotics.py](../../../tests/unit_tests/test_robotics.py) | Tests with a mocked SDK for the driver, all six directions, and cleanup |

## Usage

Follow the [installation and run guide](../../../docs/source-en/rst_source/examples/embodied/go2.rst). From the repository root, preview one direction, replacing `<interface>` with the network interface attached to the Go2:

```bash
python -m examples.embodiment.go2_sport.run --interface <interface> --direction forward --dry-run
```

The preview does not connect hardware. Direction names are `forward`, `backward`, `left`, `right`, `turn_left`, and `turn_right`. For motion, verify standing posture, clear space, and a ready remote on site, then remove `--dry-run` and enter `GO`. Each run tests one direction, with a default command duration of one second; use `--seconds 2` for two seconds. Reports go to `outputs/go2_sport/`.

### Record a State Trajectory

After the read-only check passes, choose a directory path that does not yet exist. Replace `eno1` with the robot interface and confirm motion at the prompt.

```bash
python -m examples.embodiment.go2_sport.run --interface eno1 --direction forward --record-dir outputs/go2_episodes/forward_001
```

A completed one-second run writes one file under `forward_001/pickle_episode/`. Its 10 actions are commanded `[vx, vy, yaw_rate]` in m/s, m/s and rad/s. `observations[t]` and `observations[t+1]` are the states recorded before and after command `t`. Each of the 11 observations holds attitude, angular velocity, 12 joint positions and velocities, battery voltage and four foot-force readings. The last step sets `truncated`. The filename suffix `_fail.pkl` indicates that this task supplies no success label. The JSON report keeps the timestamped SDK speed estimates in `speeds`.

### Record a Handset Demonstration

With `--handset`, RLinf reads `rt/wirelesscontroller` at each control step. It maps the stick axes to `[0.3 * ly, -0.3 * lx, -0.5 * rx]` and sends that command through the drive environment. Each axis has a 0.05 dead zone. The right stick's vertical axis is unused. The JSON report pairs each completed step with the source axes and their host reception time. The pickle stores the velocity commands.

Before using this mode, establish on a supported test stand that the handset publishes the expected axes while native stick driving is isolated, and test a stop independent of the handset input path and the host. This depends on the robot firmware and setup. The runner leaves controller settings to the operator. If moving a stick already drives the robot before RLinf starts, the setup still needs commissioning. On the stand, run the following receive-only check and inspect `steps[].mapped_velocity` in its JSON report to verify each stick direction.

```bash
python -m examples.embodiment.go2_sport.run --interface eno1 --inspect-handset --seconds 2
```

Preview the recording command with:

```bash
python -m examples.embodiment.go2_sport.run --interface eno1 --handset --seconds 2 --record-dir outputs/go2_episodes/handset_001 --dry-run
```

After checking the setup, remove `--dry-run`, center both sticks, and type `GO` on site. The first step requires centered sticks. A button press, invalid axis or input older than 0.2 s ends the run and requests a stop. A completed two-second run stores 20 actions and 21 states. An interrupted run keeps its JSON diagnostics and writes no partial pickle.

To verify both input modes on CPU, run the round-trip test from the repository root. It reads back two episodes per mode and checks the commands against the mock SDK.

```bash
PYTHONPATH=.:tests:tests/unit_tests python -m pytest -q tests/unit_tests/test_robotics.py -k go2_sport_recording_round_trip
```

## Limitations

The archive records operator-confirmed functionality. Quantitative distance, angle, and stopping-distance acceptance remains incomplete. SDK velocity estimates and stop requests do not replace direct observation; process exit and network loss require an independent stop mechanism. The environment returns zero reward and does not provide a real-robot RL task.
