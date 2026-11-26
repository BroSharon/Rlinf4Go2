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

"""Observation and motor transforms for the Go2 Isaac Lab locomotion policy."""

import math
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

# Isaac Lab groups joints by type, while SDK2 groups them by leg.
POLICY_TO_SDK = np.array([3, 0, 9, 6, 4, 1, 10, 7, 5, 2, 11, 8])
DEFAULT_JOINT_POSITIONS = np.array(
    [0.1, -0.1, 0.1, -0.1, 0.8, 0.8, 1.0, 1.0, -1.5, -1.5, -1.5, -1.5],
    dtype=np.float32,
)
PHYSICS_DT = 0.005
POLICY_DECIMATION = 4


def _vector(value: Sequence[float] | np.ndarray, size: int, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float32)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"Expected {name} as {size} finite values, got {result}.")
    return result


def _check_fresh(timestamp: float, now: float, limit: float, name: str) -> None:
    age = now - timestamp
    if not math.isfinite(age) or not 0 <= age <= limit:
        raise TimeoutError(f"Expected {name} within {limit}s, found age {age}s.")


@dataclass(frozen=True)
class JointState:
    """SDK-ordered joints and body-frame IMU values at host monotonic time."""

    q: Sequence[float]
    dq: Sequence[float]
    quaternion_wxyz: Sequence[float]
    gyroscope_b: Sequence[float]
    timestamp: float


@dataclass(frozen=True)
class Perception:
    """Measured velocity and terrain samples in the policy scan order."""

    base_lin_vel_b: Sequence[float]
    reference_z_w: float
    ground_z_w: Sequence[float]
    timestamp: float


class Go2Control:
    """Assemble the simulator policy's observations and SDK-ordered motor torques."""

    def __init__(self, task: str) -> None:
        if task not in ("A", "B", "C"):
            raise ValueError(f"Expected task A, B, or C, got {task!r}.")
        self.task = task
        self.reset()

    def reset(self) -> None:
        """Reset previous-action history and the motor tick count."""
        self.last_action = np.zeros(12, dtype=np.float32)
        self.target_q = DEFAULT_JOINT_POSITIONS.copy()
        self.steps = 0
        self.last_time = None

    @property
    def needs_action(self) -> bool:
        """Whether the next motor cycle requires a new policy action."""
        return self.steps % POLICY_DECIMATION == 0

    @staticmethod
    def scan_xy() -> np.ndarray:
        """Return the 187 yaw-aligned scan coordinates in metres."""
        x = np.linspace(-0.8, 0.8, 17, dtype=np.float32)
        y = np.linspace(-0.5, 0.5, 11, dtype=np.float32)
        xx, yy = np.meshgrid(x, y, indexing="xy")
        return np.stack((xx.ravel(), yy.ravel()), axis=-1)

    def observation(
        self,
        state: JointState,
        perception: Perception,
        command: Sequence[float] | np.ndarray,
        now: float,
    ) -> np.ndarray:
        """Return the 235-value observation from fresh measured inputs."""
        _check_fresh(state.timestamp, now, 0.05, "joint state")
        _check_fresh(perception.timestamp, now, 0.1, "terrain and velocity")
        q = _vector(state.q, 12, "SDK joint positions")[POLICY_TO_SDK]
        dq = _vector(state.dq, 12, "SDK joint velocities")[POLICY_TO_SDK]
        quaternion = _vector(state.quaternion_wxyz, 4, "wxyz quaternion")
        if not np.isclose(np.linalg.norm(quaternion), 1, atol=1e-3):
            raise ValueError("Expected a unit base-to-world quaternion.")
        w, x, y, z = quaternion
        gravity = np.array(
            [
                2 * (w * y - x * z),
                -2 * (y * z + w * x),
                -(w * w - x * x - y * y + z * z),
            ],
            dtype=np.float32,
        )
        if not math.isfinite(perception.reference_z_w):
            raise ValueError("Expected a finite terrain height reference.")
        height = np.clip(
            perception.reference_z_w
            - _vector(perception.ground_z_w, 187, "terrain samples")
            - 0.5,
            -1,
            1,
        )
        return np.concatenate(
            (
                _vector(perception.base_lin_vel_b, 3, "body velocity"),
                _vector(state.gyroscope_b, 3, "body angular velocity"),
                gravity,
                _vector(command, 3, "velocity command"),
                q - DEFAULT_JOINT_POSITIONS,
                dq,
                self.last_action,
                height,
            )
        )

    def step(
        self,
        state: JointState,
        action: Sequence[float] | np.ndarray | None,
        now: float,
    ) -> dict[str, np.ndarray]:
        """Return joint targets and torque for one 200 Hz motor cycle.

        A new policy action is required every fourth cycle. Other cycles hold
        the preceding target. Timestamps use the host monotonic clock.
        """
        _check_fresh(state.timestamp, now, 0.05, "joint state")
        if (
            self.last_time is not None
            and not 0 < now - self.last_time <= 2 * PHYSICS_DT
        ):
            raise TimeoutError("Expected advancing motor cycles within 10 ms.")
        q = _vector(state.q, 12, "SDK joint positions")[POLICY_TO_SDK]
        dq = _vector(state.dq, 12, "SDK joint velocities")[POLICY_TO_SDK]
        if self.needs_action:
            if action is None:
                raise ValueError("Expected a policy action at the 50 Hz policy tick.")
            policy_action = _vector(action, 12, "policy action")
            self.last_action = (
                np.clip(policy_action, -100, 100)
                if self.task == "B"
                else policy_action.copy()
            )
            self.target_q = DEFAULT_JOINT_POSITIONS + 0.25 * self.last_action
        elif action is not None:
            raise ValueError("Expected no policy action between policy ticks.")
        effort = 25 * (self.target_q - q) - 0.5 * dq
        velocity = np.clip(dq, -60, 60)
        torque = np.clip(
            effort,
            np.maximum(23.5 * (-1 - velocity / 30), -23.5),
            np.minimum(23.5 * (1 - velocity / 30), 23.5),
        )
        sdk_target = np.empty(12, dtype=np.float32)
        sdk_torque = np.empty(12, dtype=np.float32)
        sdk_target[POLICY_TO_SDK] = self.target_q
        sdk_torque[POLICY_TO_SDK] = torque
        self.last_time = now
        self.steps += 1
        return {"target_q_sdk": sdk_target, "torque_sdk": sdk_torque}
