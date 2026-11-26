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

"""Read the Go2 handset through the drive environment's SDK2 session."""

import threading
import time
from contextlib import ExitStack
from typing import Any, Mapping

import numpy as np

from ...actions import ActionKind
from ..base import Features
from .base import TeleopAction, TeleopDevice


@TeleopDevice.register("go2_handset")
class Go2Handset(TeleopDevice):
    """Map the original Go2 sticks to bounded body velocity commands."""

    PRODUCES = {"base": ActionKind.BASE_VELOCITY}
    AXES = ("lx", "ly", "rx", "ry")
    DEAD_ZONE = 0.05
    MAX_AGE = 0.2

    def __init__(self) -> None:
        super().__init__()
        self._lock = threading.Condition()
        self._sticks: tuple[float, np.ndarray] | None = None
        self._speed: tuple[float, np.ndarray] | None = None
        self._fault: str | None = None
        self._reset_at = float("inf")
        self._armed = False

    def _open(self) -> ExitStack:
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import (
            SportModeState_,
            WirelessController_,
        )

        with ExitStack() as subscriptions:
            sticks = ChannelSubscriber("rt/wirelesscontroller", WirelessController_)
            subscriptions.callback(sticks.Close)
            sticks.Init(self._on_sticks, 1)
            speed = ChannelSubscriber("rt/sportmodestate", SportModeState_)
            subscriptions.callback(speed.Close)
            speed.Init(self._on_speed, 1)
            return subscriptions.pop_all()

    def _on_sticks(self, message: Any) -> None:
        received = time.monotonic()
        axes = np.asarray(
            [getattr(message, axis) for axis in self.AXES], dtype=np.float32
        )
        with self._lock:
            if message.keys:
                self._fault = "Go2 handset button pressed"
            elif not np.all(np.abs(axes) <= 1):
                self._fault = "Expected four finite Go2 handset axes in [-1, 1]"
            elif (
                self._armed
                and self._sticks
                and received - self._sticks[0] > self.MAX_AGE
            ):
                self._fault = "Go2 handset input expired"
            self._sticks = received, axes
            self._lock.notify_all()

    def _on_speed(self, message: Any) -> None:
        received = time.monotonic()
        values = np.asarray([*message.velocity, message.yaw_speed], dtype=np.float32)
        if values.shape != (4,) or not np.isfinite(values).all():
            return
        with self._lock:
            if self._armed and self._speed and received - self._speed[0] > self.MAX_AGE:
                self._fault = "Go2 SDK speed feedback expired"
            self._speed = received, values
            self._lock.notify_all()

    def on_reset(self, context: Mapping[str, Any]) -> None:
        """Require recent, centered sticks before driving a new episode."""
        with self._lock:
            self._reset_at = time.monotonic()
            self._armed = False

    def _ready(self) -> bool:
        return (
            self._sticks is not None
            and self._speed is not None
            and min(self._sticks[0], self._speed[0]) >= self._reset_at
        )

    @property
    def observation_features(self) -> Features:
        """Describe raw sticks, velocity estimates and host reception times."""
        return {
            "go2_handset_axes": {"shape": (4,), "dtype": "float32"},
            "go2_handset_received_at": {"shape": (), "dtype": "float64"},
            "go2_sdk_velocity": {"shape": (3,), "dtype": "float32"},
            "go2_sdk_yaw_rate": {"shape": (), "dtype": "float64"},
            "go2_sdk_received_at": {"shape": (), "dtype": "float64"},
        }

    def get_observation(self) -> dict[str, Any]:
        """Read the handset and SDK estimates together before the control step."""
        with self._lock:
            if not self._armed:
                self._lock.wait_for(lambda: self._fault is not None or self._ready(), 3)
            if self._fault:
                raise RuntimeError(self._fault)
            if not self._ready():
                raise TimeoutError(
                    "Expected fresh Go2 handset and speed messages after reset"
                )
            sticks_at, axes = self._sticks
            speed_at, speed = self._speed
            now = time.monotonic()
            if now - sticks_at > self.MAX_AGE:
                raise TimeoutError("Go2 handset input expired")
            if now - speed_at > self.MAX_AGE:
                raise TimeoutError("Go2 SDK speed feedback expired")
            if not self._armed and np.any(np.abs(axes) > self.DEAD_ZONE):
                raise ValueError("Center both Go2 sticks before starting an episode")
            self._armed = True
        return {
            "go2_handset_axes": axes.copy(),
            "go2_handset_received_at": sticks_at,
            "go2_sdk_velocity": speed[:3].copy(),
            "go2_sdk_yaw_rate": float(speed[3]),
            "go2_sdk_received_at": speed_at,
        }

    def action(
        self, reading: Mapping[str, Any], context: Mapping[str, Any]
    ) -> TeleopAction:
        """Use the handset even at zero velocity so policy actions cannot leak in."""
        lx, ly, rx, _ = np.where(
            np.abs(reading["go2_handset_axes"]) > self.DEAD_ZONE,
            reading["go2_handset_axes"],
            0,
        )
        velocity = np.array([0.3 * ly, -0.3 * lx, -0.5 * rx], dtype=np.float32)
        return TeleopAction(parts={"base": velocity}, driving=True, info=dict(reading))
