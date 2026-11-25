# Copyright 2026 The RLinf Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Drive the Go2 from a direction or handset, or check state reception."""

import argparse
import json
import math
import os
import signal
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Literal

from rlinf.utils.logging import get_logger

COMMANDS = {
    "forward": (0.3, 0.0, 0.0),
    "backward": (-0.3, 0.0, 0.0),
    "left": (0.0, 0.3, 0.0),
    "right": (0.0, -0.3, 0.0),
    "turn_left": (0.0, 0.0, 0.5),
    "turn_right": (0.0, 0.0, -0.5),
}
logger = get_logger()


class Handset:
    """Sample the latest wireless controller input once per control step."""

    def __init__(self) -> None:
        self.ready = threading.Event()
        self.lock = threading.Lock()
        self.sample: dict[str, Any] = {}
        self.error = ""

    def callback(self, message: Any) -> None:
        """Latch buttons and invalid inputs even between control steps."""
        with self.lock:
            now = time.monotonic()
            try:
                axes = {k: float(getattr(message, k)) for k in ("lx", "ly", "rx", "ry")}
                if not all(-1 <= v <= 1 for v in axes.values()):
                    raise ValueError("Expected four finite handset axes within [-1, 1]")
                if message.keys != 0:
                    raise ValueError("Handset button pressed")
                if self.sample and now - self.sample["t"] > 0.2:
                    raise ValueError("Handset input expired")
                self.sample = {"t": now, **axes}
            except (AttributeError, TypeError, ValueError, OverflowError) as exc:
                self.error = str(exc)
            self.ready.set()

    def command(self, *, require_neutral: bool) -> tuple[list[float], dict[str, Any]]:
        """Return a bounded body velocity and its timestamped source axes."""
        with self.lock:
            if self.error:
                raise ValueError(self.error)
            sample = self.sample.copy()
        if not sample or time.monotonic() - sample["t"] > 0.2:
            raise TimeoutError("Handset input expired")
        if require_neutral and any(
            abs(sample[k]) > 0.05 for k in ("lx", "ly", "rx", "ry")
        ):
            raise ValueError("Center both sticks before starting handset control")
        command = [
            0.0 if abs(sample[k]) <= 0.05 else sample[k] * scale
            for k, scale in (("ly", 0.3), ("lx", -0.3), ("rx", -0.5))
        ]
        return command, sample


class SpeedLog:
    """Record SDK velocity estimates using host reception timestamps."""

    def __init__(self) -> None:
        self.rows: list[dict[str, Any]] = []
        self.lock = threading.Lock()

    def callback(self, state: Any) -> None:
        """Ignore malformed or nonfinite SDK estimates."""
        try:
            velocity = [float(v) for v in state.velocity]
            yaw = float(state.yaw_speed)
            if len(velocity) != 3 or not all(map(math.isfinite, velocity + [yaw])):
                return
        except (AttributeError, TypeError, ValueError, OverflowError):
            return
        with self.lock:
            self.rows.append(
                {"t": time.monotonic(), "velocity": velocity, "yaw_speed": yaw}
            )

    def snapshot(self) -> list[dict[str, Any]]:
        """Return the received samples in time order."""
        with self.lock:
            return list(self.rows)

    def fresh(self) -> bool:
        """Require a sample received within 0.2 seconds."""
        rows = self.snapshot()
        return bool(rows) and time.monotonic() - rows[-1]["t"] <= 0.2

    def stopped(self, since: float) -> bool:
        """Check SDK estimates for 0.5 seconds; physical confirmation is separate."""
        rows = [r for r in self.snapshot() if r["t"] >= since]
        if len(rows) < 3 or time.monotonic() - rows[-1]["t"] > 0.2:
            return False
        start = previous = rows[-1]["t"]
        for row in reversed(rows):
            if previous - row["t"] > 0.2:
                return False
            previous = row["t"]
            if math.hypot(*row["velocity"][:2]) > 0.05 or abs(row["yaw_speed"]) > 0.1:
                return False
            if start - row["t"] >= 0.5:
                return True
        return False


def competing_processes() -> list[int]:
    """Find other local Go2 Python scripts; remote controllers are not visible."""
    matches = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            argv = [
                os.fsdecode(a) for a in (entry / "cmdline").read_bytes().split(b"\0")
            ]
            if not argv or "python" not in Path(argv[0]).name:
                continue
            scripts = [a for a in argv[1:] if a.endswith(".py")]
            modules = [argv[i + 1] for i, a in enumerate(argv[:-1]) if a == "-m"]
            if any("go2" in Path(a).name.lower() for a in scripts + modules):
                matches.append(int(entry.name))
        except (OSError, ValueError):
            continue
    return matches


def run_check(
    interface: str,
    direction: str | None,
    seconds: int,
    record_dir: Path | None = None,
    *,
    handset: Literal["drive", "inspect"] | None = None,
) -> dict[str, Any]:
    """Run a bounded check after the caller obtains operator confirmation.

    Args:
        interface: Network interface attached to the Go2.
        direction: Scripted direction. Omit for handset or read-only operation.
        seconds: One or two seconds at ten steps per second.
        record_dir: New directory for the motion episode.
        handset: Inspect mapped inputs with motion disabled, or drive from them.
            Driving requires verified exclusive SDK control and an independent stop.
    """
    if seconds not in (1, 2) or (direction is not None and direction not in COMMANDS):
        raise ValueError("Expected a named direction or None, and one or two seconds")
    if handset and direction is not None:
        raise ValueError("Choose either a scripted direction or handset input")
    if handset not in (None, "inspect", "drive"):
        raise ValueError("Expected handset mode 'inspect' or 'drive'")
    motion = direction is not None or handset == "drive"
    if record_dir is not None and not motion:
        raise ValueError("Episode recording requires motion input")

    import gymnasium as gym
    import numpy as np

    from rlinf.robotics import Go2Config, RobotDiscovery

    steps = seconds * 10
    report = {
        "interface": interface,
        "direction": direction,
        "command": COMMANDS[direction] if direction else None,
        "planned_seconds": seconds,
        "step_frequency_hz": 10,
        "uses_ppo": False,
        "operator_confirmed_motion_and_stop": None,
        "sdk_stop_estimate": "not_checked",
        "errors": [],
        "steps": [],
    }
    env = subscriber = handset_subscriber = None
    speeds = SpeedLog()
    try:
        if record_dir is not None:
            from rlinf.envs.real.venv import NoAutoResetSyncVectorEnv
            from rlinf.envs.wrappers import CollectEpisode

            record_dir.mkdir(parents=True, exist_ok=False)
            report["record_dir"] = str(record_dir)
        cfg = Go2Config(node_rank=0, interface=interface, enable_motion=motion)
        resource = RobotDiscovery.registry["UnitreeGo2"].discovery_cls.enumerate(
            0, [cfg]
        )
        env = gym.make(
            "rlinf.envs.real.go2:Go2DriveEnv-v1",
            override_cfg={"step_frequency": 10.0, "max_num_steps": steps},
            worker_info=None,
            robot_info=resource.infos[0],
            env_idx=0,
            env_cfg={},
        )
        if record_dir is not None:
            drive_env = env
            # Keep the environment axis distinct from battery_voltage's (1,) axis.
            env = NoAutoResetSyncVectorEnv([lambda: drive_env])
            observation_space = env.observation_space
            env = CollectEpisode(
                env,
                save_dir=str(record_dir),
                robot_type="unitree_go2",
                export_format="pickle_episode",
                only_success=False,
                show_goal_site=False,
            )
        else:
            observation_space = env.observation_space
        if motion or handset:
            from unitree_sdk2py.core.channel import ChannelSubscriber
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

            subscriber = ChannelSubscriber("rt/sportmodestate", SportModeState_)
            subscriber.Init(speeds.callback, 10)
        if motion:
            deadline = time.monotonic() + 3
            while not speeds.fresh() and time.monotonic() < deadline:
                time.sleep(0.02)
            if not speeds.fresh():
                raise TimeoutError("No fresh SportModeState; no Move will be sent")
        if handset:
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import WirelessController_

            report["input_source"] = "rt/wirelesscontroller"
            report["handset_mode"] = handset
            controller = Handset()
            handset_subscriber = ChannelSubscriber(
                report["input_source"], WirelessController_
            )
            handset_subscriber.Init(controller.callback, 10)
            if not controller.ready.wait(3):
                raise TimeoutError("No wireless controller input within three seconds")
        observation, _ = env.reset()
        for index in range(steps):
            row = {}
            if handset:
                command, row["handset"] = controller.command(
                    require_neutral=motion and index == 0
                )
                if not motion:
                    row["mapped_velocity"] = command
            if motion:
                if not speeds.fresh():
                    raise TimeoutError("Speed feedback expired; requesting stop")
                if direction:
                    command = COMMANDS[direction]
                action = np.asarray(command, dtype=np.float32)
                if handset:
                    row["command"] = action.tolist()
                observation, _, terminated, truncated, _ = env.step(
                    action[None, :] if record_dir is not None else action
                )
            else:
                observation, _ = env.reset()
                time.sleep(0.1)
                terminated = truncated = False
            row.update(
                t=time.monotonic(),
                observation_valid=observation_space.contains(observation),
            )
            report["steps"].append(row)
            if not report["steps"][-1]["observation_valid"]:
                raise ValueError("Received state outside the Go2 observation space")
            if terminated or truncated:
                break
    except (Exception, KeyboardInterrupt) as exc:
        report["errors"].append(repr(exc))
    finally:
        try:
            if env is not None:
                env.close()
                report["close_completed"] = True
        except (Exception, KeyboardInterrupt) as exc:
            report["errors"].append("close: " + repr(exc))
        try:
            if motion and subscriber is not None:
                logger.warning(
                    "Stop requested. Use the remote if the robot is still moving."
                )
                since = time.monotonic()
                deadline = since + 2
                while not speeds.stopped(since) and time.monotonic() < deadline:
                    time.sleep(0.02)
                report["sdk_stop_estimate"] = (
                    "below_thresholds" if speeds.stopped(since) else "unconfirmed"
                )
        except (Exception, KeyboardInterrupt) as exc:
            report["errors"].append("stop observation: " + repr(exc))
        finally:
            for channel in (handset_subscriber, subscriber):
                if channel is not None:
                    try:
                        channel.Close()
                    except Exception as exc:
                        report["errors"].append("subscriber close: " + repr(exc))
            report["speeds"] = speeds.snapshot()
    return report


def main() -> None:
    """Preview by flag, or obtain confirmation before opening SDK channels."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", required=True)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--direction", choices=COMMANDS)
    mode.add_argument("--readonly", action="store_true")
    mode.add_argument(
        "--handset",
        action="store_const",
        const="drive",
        help="Drive from the wireless sticks",
    )
    mode.add_argument(
        "--inspect-handset",
        dest="handset",
        action="store_const",
        const="inspect",
        help="Read sticks and preview mapped velocities with SDK motion disabled",
    )
    parser.add_argument("--seconds", type=int, choices=(1, 2), default=1)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--record-dir", type=Path, help="Save the motion episode under a new directory"
    )
    args = parser.parse_args()
    motion = args.direction is not None or args.handset == "drive"
    if args.record_dir is not None and not motion:
        parser.error("--record-dir requires --direction or --handset")
    plan = {
        "interface": args.interface,
        "direction": args.direction,
        "handset": args.handset,
        "command": COMMANDS.get(args.direction),
        "seconds": args.seconds,
        "hardware_access": False,
    }
    logger.info("Check plan: %s", json.dumps(plan))
    if args.dry_run:
        return
    if motion:
        conflicts = competing_processes()
        if conflicts:
            raise RuntimeError(f"Other local Go2 Python processes found: {conflicts}")
        prompt = "Standing, area clear, remote in hand: type GO: "
        if args.handset:
            prompt = (
                "Confirm on site: standing, area clear, sticks centered, "
                "native stick driving isolated and independent stop tested. Type GO: "
            )
        else:
            logger.warning(
                "Keep the original controller active and the remote ready. Stop timing is not guaranteed."
            )
        try:
            if input(prompt).strip() != "GO":
                return
        except (EOFError, KeyboardInterrupt):
            return
    output = args.output or Path("outputs/go2_sport") / (
        (args.direction or (f"handset_{args.handset}" if args.handset else "readonly"))
        + "_"
        + datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        + ".json"
    )
    output.parent.mkdir(parents=True, exist_ok=True)

    def interrupted(signum: int, frame: Any) -> None:
        raise KeyboardInterrupt("SIGTERM received")

    with output.open("x") as stream:
        old_handler = signal.signal(signal.SIGTERM, interrupted)
        try:
            report = run_check(
                args.interface,
                args.direction,
                args.seconds,
                args.record_dir,
                handset=args.handset,
            )
            json.dump(report, stream, indent=2, allow_nan=False)
            stream.write("\n")
        finally:
            signal.signal(signal.SIGTERM, old_handler)
    logger.info("Report: %s; physical result requires operator confirmation", output)
    if report["errors"] or (
        motion and report["sdk_stop_estimate"] != "below_thresholds"
    ):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
