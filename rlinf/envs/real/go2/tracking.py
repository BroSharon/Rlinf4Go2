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

"""One bounded episode of Sport controller velocity tracking."""

import json
import platform
import shutil
import subprocess
import time
from importlib.metadata import PackageNotFoundError, version
from pathlib import Path
from typing import Any, Mapping
from uuid import uuid4

import numpy as np

from rlinf.robotics import Go2Config, RobotInfo
from rlinf.robotics.parts.mobility.unitree_go2 import Go2Base
from rlinf.scheduler import WorkerInfo

from .drive import Go2DriveEnv


def pair_step(events: list[dict], end_ns: int) -> dict:
    """Select the last finite Sport estimate in this command's half-open window."""
    commands = [event for event in events if event["type"] == "dispatch"]
    dispatch = commands[0] if commands else None
    row = {
        "dispatch": dispatch,
        "end_ns": end_ns,
        "feedback": None,
        "status": "not_dispatched",
    }
    if dispatch is None:
        return row
    start = dispatch["time_ns"]
    end = min(
        [end_ns]
        + [event["time_ns"] for event in commands[1:]]
        + [
            event["time_ns"]
            for event in events
            if event["type"] == "stop" and event["time_ns"] >= start
        ]
    )
    row["end_ns"] = end
    if dispatch["status"] != "dispatched":
        return row
    samples = (
        event
        for event in events
        if event["type"] == "feedback"
        and start <= event["time_ns"] < end
        and np.isfinite([*event["velocity"], event["yaw_speed"]]).all()
    )
    row["feedback"] = max(samples, key=lambda e: e["time_ns"], default=None)
    row["status"] = "valid" if row["feedback"] is not None else "missing"
    return row


def episode_metrics(
    steps: list[dict], *, complete: bool, mapping_confirmed: bool
) -> dict[str, float | int]:
    """Compute coverage over dispatched steps and MAE for eligible episodes."""
    dispatched = sum(row["status"] != "not_dispatched" for row in steps)
    valid = [row for row in steps if row["status"] == "valid"]
    eligible = bool(valid) and complete and mapping_confirmed
    metrics = {
        "go2_tracking_dispatched_steps": dispatched,
        "go2_tracking_valid_steps": len(valid),
        "go2_tracking_valid_episode": int(eligible),
    }
    if dispatched:
        metrics["go2_tracking_feedback_coverage"] = len(valid) / dispatched
    if eligible:
        errors = []
        for row in valid:
            feedback = row["feedback"]
            estimate = [*feedback["velocity"][:2], feedback["yaw_speed"]]
            errors.append(np.abs(np.subtract(estimate, row["dispatch"]["command"])))
        for axis, mae in zip(
            ("vx_mps", "vy_mps", "yaw_rate_radps"), np.mean(errors, axis=0)
        ):
            metrics[f"go2_tracking_mae_{axis}"] = float(mae)
    return metrics


class Go2TrackingEnv(Go2DriveEnv):
    """Pair Sport estimates on the robot host and publish terminal tracking metrics."""

    TRACK_VELOCITY = True

    @classmethod
    def validate_env_config(
        cls, override_cfg: Mapping[str, Any], env_cfg: Mapping[str, Any]
    ) -> list[Any]:
        """Require one bounded rollout that reaches the Go2 stop boundary."""
        devices = super().validate_env_config(override_cfg, env_cfg)
        steps = override_cfg["max_num_steps"]
        for key, expected in (
            ("total_num_envs", 1),
            ("rollout_epoch", 1),
            ("max_steps_per_rollout_epoch", steps),
            ("max_episode_steps", steps),
        ):
            if env_cfg.get(key) != expected:
                raise ValueError(
                    f"Go2 tracking requires {key}={expected}, got {env_cfg.get(key)}."
                )
        return devices

    def __init__(
        self,
        override_cfg: dict[str, Any],
        worker_info: WorkerInfo | None = None,
        robot_info: RobotInfo[Go2Config] | None = None,
        env_idx: int = 0,
    ) -> None:
        cfg = dict(override_cfg)
        self._log_dir = Path(cfg.pop("log_dir"))
        self._log_dir.mkdir(exist_ok=True, parents=True)
        self._command_name = cfg.pop("command_name")
        self._mapping_source = cfg.pop("mapping_source", None)
        if self._mapping_source is not None and (
            not isinstance(self._mapping_source, str)
            or not self._mapping_source.strip()
        ):
            raise ValueError(
                f"Expected a nonempty mapping confirmation source or null, got {self._mapping_source!r}."
            )
        self._versions = {"python": platform.python_version()}
        try:
            self._versions["rlinf"] = version("rlinf")
        except PackageNotFoundError:
            pass
        if shutil.which("git"):
            rev = subprocess.run(
                ["git", "describe", "--always", "--dirty", "--abbrev=40"],
                cwd=Path(__file__).parent,
                capture_output=True,
                text=True,
            )
            if rev.returncode == 0:
                self._versions["git_revision"] = rev.stdout.strip()
        self._source = "dummy" if cfg.get("is_dummy") else "unavailable"
        self._steps: list[dict] = []
        self._events: list[dict] = []
        try:
            super().__init__(cfg, worker_info, robot_info, env_idx)
        except Exception as exc:
            self._finish(f"{type(exc).__name__}: {exc}")
            raise

    def _stop(self) -> None:
        if self.config.is_dummy:
            self._events.append(
                {"type": "stop", "time_ns": time.monotonic_ns(), "status": "requested"}
            )
        super()._stop()

    def _drain(self) -> tuple[int, list[dict]]:
        if self.robot is None:
            return time.monotonic_ns(), []
        batch = self.robot.child("base", Go2Base).drain_tracking()
        self._source = batch["source"]
        self._versions["sdk2"] = batch["sdk_version"]
        self._events.extend(
            e for e in batch["events"] if e["type"] in ("fault", "stop")
        )
        return batch["end_ns"], batch["events"]

    def _drive(self, velocity: np.ndarray) -> dict[str, Any]:
        if self.robot is None:
            start = time.monotonic_ns()
        obs = super()._drive(velocity)
        if self.robot is None:
            events = [
                {
                    "type": "dispatch",
                    "time_ns": start,
                    "command": velocity.tolist(),
                    "status": "dispatched",
                },
                {
                    "type": "feedback",
                    "time_ns": time.monotonic_ns(),
                    "velocity": [*velocity[:2].tolist(), 0.0],
                    "yaw_speed": float(velocity[2]),
                },
            ]
            end = time.monotonic_ns()
        else:
            end, events = self._drain()
        self._steps.append(pair_step(events, end))
        return obs

    def _finish(self, reason: str) -> dict[str, float | int]:
        source = self._mapping_source
        if self._source in ("dummy", "mock"):
            source = f"Known {self._source} body-frame feedback"
        fault = next((e for e in self._events if e["type"] == "fault"), None)
        if fault and reason == "max_num_steps":
            reason = fault["error"]
        metrics = episode_metrics(
            self._steps,
            complete=reason == "max_num_steps",
            mapping_confirmed=source is not None,
        )
        evidence = {
            "software_versions": self._versions,
            "feedback_source": self._source,
            "command_name": self._command_name,
            "units": ["m/s", "m/s", "rad/s"],
            "mapping": ["vx=velocity[0]", "vy=velocity[1]", "yaw_rate=yaw_speed"],
            "mapping_source": source,
            "clock": "robot host monotonic_ns",
            "end_reason": reason,
            "steps": self._steps,
            "events": self._events,
            "metrics": metrics,
        }
        path = self._log_dir / f"go2_tracking_{uuid4().hex}.json"
        path.write_text(json.dumps(evidence, allow_nan=False, indent=2) + "\n")
        self._steps.clear()
        self._events.clear()
        return metrics

    def reset(
        self, *, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        """Start new windows after the robot stops."""
        try:
            result = super().reset(seed=seed, options=options)
        except BaseException as exc:
            self._drain()
            self._finish(f"{type(exc).__name__}: {exc}")
            raise
        self._drain()
        if self._steps:
            self._finish("reset")
        self._events.clear()
        return result

    def step(
        self, action: np.ndarray
    ) -> tuple[dict[str, Any], float, bool, bool, dict[str, Any]]:
        """Return metrics at truncation and preserve interrupted episodes on disk."""
        before = len(self._steps)
        try:
            obs, reward, terminated, truncated, info = super().step(action)
        except BaseException as exc:
            end, events = self._drain()
            if len(self._steps) == before:
                self._steps.append(pair_step(events, end))
            self._finish(f"{type(exc).__name__}: {exc}")
            raise
        if truncated:
            self._drain()
            info["episode"] = self._finish("max_num_steps")
        return obs, reward, terminated, truncated, info

    def close(self) -> None:
        """Save partial episodes after requesting stop and release both subscribers."""
        try:
            if self._steps:
                try:
                    self._stop()
                finally:
                    self._drain()
                    self._finish("closed")
        finally:
            super().close()
