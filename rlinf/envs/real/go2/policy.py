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

"""Go2 policy evaluation using a simulated clock and mock SDK."""

from typing import Any, Mapping

import gymnasium as gym
import numpy as np

from rlinf.envs.real.go2.control import (
    DEFAULT_JOINT_POSITIONS,
    PHYSICS_DT,
    POLICY_DECIMATION,
    POLICY_TO_SDK,
    Go2Control,
    JointState,
    Perception,
)
from rlinf.envs.real.utils.config import get_hardware_config
from rlinf.envs.real.wrappers.teleop.config import resolve_teleop_devices
from rlinf.robotics import Go2Config, RobotInfo
from rlinf.robotics.parts.mobility.unitree_go2_lowlevel import Go2LowLevel
from rlinf.scheduler import WorkerInfo


class Go2PolicyEnv(gym.Env):
    """Evaluate Go2 locomotion actions against simulated sensor inputs."""

    TELEOP = ()
    TELEOP_DEFAULT = "none"
    ACTION_WRAPPERS = ()
    TRANSFORMS = ()

    @classmethod
    def validate_env_config(
        cls, override_cfg: Mapping[str, Any], env_cfg: Mapping[str, Any]
    ) -> list[Any]:
        """Require one bounded evaluation episode."""
        steps = override_cfg["max_num_steps"]
        for key, value in (
            ("total_num_envs", 1),
            ("rollout_epoch", 1),
            ("max_episode_steps", steps),
            ("max_steps_per_rollout_epoch", steps),
            ("auto_reset", False),
        ):
            if env_cfg.get(key) != value:
                raise ValueError(f"Expected {key}={value}, got {env_cfg.get(key)}.")
        return resolve_teleop_devices(
            env_cfg, supported=cls.TELEOP, default=cls.TELEOP_DEFAULT
        )

    def __init__(
        self,
        override_cfg: dict[str, Any],
        worker_info: WorkerInfo | None = None,
        robot_info: RobotInfo[Go2Config] | None = None,
        env_idx: int = 0,
    ) -> None:
        self.is_dummy = override_cfg["is_dummy"]
        self.max_num_steps = override_cfg["max_num_steps"]
        if not isinstance(self.max_num_steps, int) or self.max_num_steps <= 0:
            raise ValueError(
                f"Expected positive max_num_steps, got {self.max_num_steps}."
            )
        self.command = np.asarray(override_cfg["command"], dtype=np.float32)
        if self.command.shape != (3,) or not np.isfinite(self.command).all():
            raise ValueError(
                f"Expected finite velocity command with shape (3,), got {self.command}."
            )
        self.controller = Go2Control(override_cfg["task"])
        self.action_space = gym.spaces.Box(-np.inf, np.inf, (12,), dtype=np.float32)
        self.observation_space = gym.spaces.Dict(
            {
                "state": gym.spaces.Dict(
                    {
                        "policy": gym.spaces.Box(
                            -np.inf, np.inf, (235,), dtype=np.float32
                        )
                    }
                )
            }
        )
        self.robot = None
        self.closed = False
        self.needs_reset = True
        self.steps = 0
        self.clock = 0.0
        if not self.is_dummy:
            import unitree_sdk2py

            if getattr(unitree_sdk2py, "__version__", None) != "mock":
                raise RuntimeError(
                    "Go2 policy evaluation currently requires the mock SDK; "
                    "real-time motor timing and perception remain to be validated."
                )
            hardware = get_hardware_config(Go2Config, robot_info, is_dummy=False)
            self.robot = Go2LowLevel(hardware.interface, enable_motion=True)
            try:
                self.robot.connect()
            except BaseException:
                self.robot.disconnect()
                raise

    @property
    def task_description(self) -> str:
        """Describe the task to the rollout worker."""
        return "evaluate Go2 locomotion policy with mock sensors"

    def _state(self) -> JointState:
        if self.robot is None:
            q = np.empty(12, dtype=np.float32)
            q[POLICY_TO_SDK] = DEFAULT_JOINT_POSITIONS
            dq = np.zeros(12, dtype=np.float32)
            quaternion = [1.0, 0.0, 0.0, 0.0]
            gyroscope = [0.0, 0.0, 0.0]
        else:
            state = self.robot.get_observation()
            q, dq = state["joint_position"], state["joint_velocity"]
            quaternion, gyroscope = state["quaternion"], state["gyroscope"]
        return JointState(q, dq, quaternion, gyroscope, self.clock)

    def _observation(self) -> dict[str, Any]:
        perception = Perception([0.0, 0.0, 0.0], 0.4, [0.0] * 187, self.clock)
        observation = self.controller.observation(
            self._state(), perception, self.command, self.clock
        )
        return {"state": {"policy": observation}}

    def reset(
        self, *, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        """Reset the mock control history."""
        if self.closed:
            raise RuntimeError("Expected an open Go2 policy environment.")
        super().reset(seed=seed)
        self.controller.reset()
        self.steps = 0
        self.clock = 0.0
        self.needs_reset = False
        return self._observation(), {}

    def step(
        self, action: np.ndarray
    ) -> tuple[dict[str, Any], float, bool, bool, dict[str, Any]]:
        """Apply one policy action across four simulated motor cycles."""
        if self.closed or self.needs_reset:
            raise RuntimeError("Reset an open Go2 policy environment before stepping.")
        try:
            for tick in range(POLICY_DECIMATION):
                result = self.controller.step(
                    self._state(), action if tick == 0 else None, self.clock
                )
                if self.robot is not None:
                    self.robot.send_action({"torque": result["torque_sdk"]})
                self.clock += PHYSICS_DT
            self.steps += 1
            truncated = self.steps >= self.max_num_steps
            self.needs_reset = truncated
            return (
                self._observation(),
                0.0,
                False,
                truncated,
                {
                    "requires_reset": truncated,
                    "clock": "simulated",
                },
            )
        except BaseException:
            self.needs_reset = True
            self.close()
            raise

    def close(self) -> None:
        """Release the mock SDK connection."""
        if self.closed:
            return
        self.closed = True
        self.needs_reset = True
        if self.robot is not None:
            self.robot.disconnect()
            self.robot = None
