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

"""Go2 SDK2 low-level state and torque transport."""

import threading
import time
from contextlib import ExitStack
from typing import Any

import numpy as np

from rlinf.robotics.go2_dds import go2_dds_session

from ..base import Action, Features, Observation
from .base import MobileBase


@MobileBase.register("unitree_go2_lowlevel")
class Go2LowLevel(MobileBase):
    """Read checksum-validated state and optionally write motor torques.

    Torque control requires a separately selected robot mode. Closing the
    publisher cannot establish that the robot is supported or stationary.
    """

    def __init__(
        self,
        interface: str,
        *,
        enable_motion: bool = False,
        state_timeout: float = 0.05,
    ) -> None:
        if not isinstance(enable_motion, bool):
            raise TypeError("Expected enable_motion to be a boolean.")
        if not interface.strip():
            raise ValueError("Expected an explicit Go2 network interface.")
        if not np.isfinite(state_timeout) or state_timeout <= 0:
            raise ValueError(
                f"Expected positive finite state_timeout, got {state_timeout}."
            )
        self.interface = interface
        self.enable_motion = enable_motion
        self.state_timeout = state_timeout
        self._resources = ExitStack()
        self._state_lock = threading.Lock()
        self._send_lock = threading.Lock()
        self._ready = threading.Event()
        self._state: Observation | None = None
        self._tick: int | None = None
        self._publisher: Any = None
        self._fault: str | None = None

    @property
    def observation_features(self) -> Features:
        return {
            "joint_position": {"shape": (12,), "dtype": "float32"},
            "joint_velocity": {"shape": (12,), "dtype": "float32"},
            "quaternion": {"shape": (4,), "dtype": "float32"},
            "gyroscope": {"shape": (3,), "dtype": "float32"},
            "timestamp": {"shape": (), "dtype": "float64"},
        }

    @property
    def action_features(self) -> Features:
        return {"torque": {"shape": (12,), "dtype": "float32"}}

    def _receive(self, message: Any) -> None:
        try:
            if message.crc != self._crc.Crc(message):
                return
            values = {
                "joint_position": np.asarray(
                    [m.q for m in message.motor_state[:12]], dtype=np.float32
                ),
                "joint_velocity": np.asarray(
                    [m.dq for m in message.motor_state[:12]], dtype=np.float32
                ),
                "quaternion": np.asarray(
                    message.imu_state.quaternion, dtype=np.float32
                ),
                "gyroscope": np.asarray(message.imu_state.gyroscope, dtype=np.float32),
            }
            if any(
                values[name].shape != shape or not np.isfinite(values[name]).all()
                for name, shape in (
                    ("joint_position", (12,)),
                    ("joint_velocity", (12,)),
                    ("quaternion", (4,)),
                    ("gyroscope", (3,)),
                )
            ):
                return
        except (AttributeError, TypeError, ValueError):
            return
        with self._state_lock:
            if self._tick == message.tick:
                return
            self._tick = message.tick
            values["timestamp"] = time.monotonic()
            self._state = values
        self._ready.set()

    def _open(self) -> Any:
        from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
        from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_
        from unitree_sdk2py.utils.crc import CRC

        self._resources.enter_context(go2_dds_session(self.interface))
        try:
            self._crc = CRC()
            self._ready.clear()
            self._state = None
            self._tick = None
            self._fault = None
            subscriber = ChannelSubscriber("rt/lowstate", LowState_)
            self._resources.callback(subscriber.Close)
            subscriber.Init(self._receive, 1)
            if not self._ready.wait(self.state_timeout):
                raise TimeoutError(
                    "Expected a checksum-valid Go2 LowState before connection."
                )
            if self.enable_motion:
                from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
                    MotionSwitcherClient,
                )

                switcher = MotionSwitcherClient()
                switcher.SetTimeout(1.0)
                switcher.Init()
                code, mode = switcher.CheckMode()
                if code != 0 or mode is None or mode["name"]:
                    raise RuntimeError(
                        f"Expected inactive high-level motion service, got {code}, {mode}."
                    )
                self._message = unitree_go_msg_dds__LowCmd_()
                self._message.head[:] = [0xFE, 0xEF]
                self._message.level_flag = 0xFF
                self._message.gpio = 0
                for motor in self._message.motor_cmd:
                    motor.mode = 0x01
                    motor.q = 2.146e9
                    motor.dq = 16000.0
                    motor.kp = motor.kd = motor.tau = 0.0
                self._publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
                self._resources.callback(self._publisher.Close)
                self._publisher.Init()
            return subscriber
        except BaseException:
            self._resources.close()
            self._publisher = None
            raise

    def get_observation(self) -> Observation:
        """Return the newest fresh, checksum-valid SDK-order motor state."""
        if not self.is_connected:
            raise RuntimeError("Connect Go2 before reading state.")
        with self._state_lock:
            state = self._state
            if (
                state is None
                or time.monotonic() - state["timestamp"] > self.state_timeout
            ):
                self._fault = "Go2 LowState expired; reconnect before sending torque."
                raise TimeoutError(self._fault)
            return {
                key: value.copy() if isinstance(value, np.ndarray) else value
                for key, value in state.items()
            }

    def send_action(self, action: Action) -> Observation:
        """Publish twelve bounded motor torques in SDK joint order."""
        if not self.is_connected:
            raise RuntimeError("Connect Go2 before sending torque.")
        with self._send_lock:
            if not self.enable_motion or self._publisher is None:
                raise PermissionError("Expected explicit low-level motion opt-in.")
            if self._fault is not None:
                raise RuntimeError(self._fault)
            if set(action) != {"torque"}:
                raise ValueError("Expected only torque in the low-level action.")
            torque = np.asarray(action["torque"], dtype=np.float32)
            if (
                torque.shape != (12,)
                or not np.isfinite(torque).all()
                or np.any(np.abs(torque) > 23.5)
            ):
                raise ValueError("Expected twelve finite SDK torques within 23.5 Nm.")
            self.get_observation()
            for motor, value in zip(self._message.motor_cmd, torque):
                motor.tau = float(value)
            self._message.crc = self._crc.Crc(self._message)
            if not self._publisher.Write(self._message, timeout=0.01):
                self._fault = (
                    "Go2 LowCmd write failed; reconnect before sending torque."
                )
                raise OSError(self._fault)
            return {"torque": torque.copy()}

    def _release(self, device: Any) -> None:
        with self._send_lock:
            self._publisher = None
            self._resources.close()
