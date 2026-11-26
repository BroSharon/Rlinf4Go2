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

"""In-memory SDK2 edge fake; no sockets, DDS participants, or robot access."""

import json
import threading
from contextlib import contextmanager
from pathlib import Path
from types import ModuleType, SimpleNamespace
from typing import Any


def lowstate(tick: int = 1) -> SimpleNamespace:
    """Return deterministic Go2 lowstate data with twelve joints."""
    return SimpleNamespace(
        tick=tick,
        crc=42,
        imu_state=SimpleNamespace(
            quaternion=[1.0, 0.0, 0.0, 0.0],
            rpy=[0.1, 0.2, 0.3],
            gyroscope=[1, 2, 3],
        ),
        motor_state=[SimpleNamespace(q=i / 10, dq=i / 20) for i in range(12)],
        power_v=31.52,
        foot_force=[40, 0, 0, 0],
    )


class FakeGo2SDK:
    """Publish state on subscription and record every motion/stop call."""

    def __init__(
        self, *, stream_feedback: bool = False, trace_path: str | None = None
    ) -> None:
        self.stream_feedback = stream_feedback
        self.trace_path = Path(trace_path) if trace_path else None
        self.publish_initial = True
        self.stop_code = 0
        self.move_code = 0
        self.move_error = False
        self.events: list[tuple[Any, ...]] = []
        self.writes: list[tuple[object, str, float]] = []
        self.stopped = threading.Event()
        self.close_entered = threading.Event()
        self.close_release = threading.Event()
        self.close_release.set()
        self.callback: Any = None
        self.sport_callback: Any = None
        self.publish_sport = True
        self.motion_mode = ""
        self.lowcmd_result = True

    def record(self, event: tuple[Any, ...]) -> None:
        """Keep SDK calls available to in-process and distributed assertions."""
        self.events.append(event)
        if self.trace_path is not None:
            with self.trace_path.open("a") as trace:
                trace.write(json.dumps(event) + "\n")

    def publish(self) -> None:
        """Deliver one recent state through the real adapter's SDK callback."""
        self.callback(lowstate())

    @contextmanager
    def streaming(self):
        """Deliver both SDK state topics while a manual check runs."""
        shutdown = threading.Event()

        def publish() -> None:
            while not shutdown.wait(0.01):
                if self.callback is not None:
                    self.publish()
                if self.publish_sport and self.sport_callback is not None:
                    self.sport_callback(
                        SimpleNamespace(velocity=[0.0, 0.0, 0.0], yaw_speed=0.0)
                    )

        thread = threading.Thread(target=publish, daemon=True)
        thread.start()
        try:
            yield self
        finally:
            shutdown.set()
            thread.join()

    def modules(self) -> dict[str, ModuleType]:
        """Return importable SDK modules for an isolated test process."""
        fake = self

        class Subscriber:
            def __init__(self, topic: str, dtype: type) -> None:
                self.topic = topic
                self.shutdown = threading.Event()
                self.thread = None
                fake.record(("subscribe", topic))

            def Init(self, callback: Any, queue_len: int) -> None:
                if self.topic == "rt/sportmodestate":
                    fake.sport_callback = callback
                    return
                fake.callback = callback
                if fake.publish_initial:
                    fake.publish()
                if fake.stream_feedback:

                    def stream_state() -> None:
                        tick = 1
                        while not self.shutdown.wait(0.01):
                            tick += 1
                            callback(lowstate(tick))

                    self.thread = threading.Thread(target=stream_state, daemon=True)
                    self.thread.start()

            def Close(self) -> None:
                if self.topic == "rt/sportmodestate":
                    fake.sport_callback = None
                    fake.record(("sport_close",))
                    return
                if self.thread is not None:
                    self.shutdown.set()
                    self.thread.join()
                fake.close_entered.set()
                if not fake.close_release.wait(2):
                    raise TimeoutError("Test did not release subscriber close")
                fake.record(("close",))

        class Client:
            def __init__(self) -> None:
                self.writer = object()
                fake.record(("client",))

            def SetTimeout(self, timeout: float) -> None:
                self.timeout = timeout

            def Init(self) -> None:
                fake.record(("init",))

            def StopMove(self) -> int:
                fake.writes.append((self.writer, "stop", self.timeout))
                fake.record(("stop",))
                fake.stopped.set()
                return fake.stop_code

            def Move(self, vx: float, vy: float, vyaw: float) -> int:
                fake.writes.append((self.writer, "move", self.timeout))
                fake.record(("move", vx, vy, vyaw))
                if fake.move_error:
                    raise OSError("Simulated DDS send failure")
                if (
                    fake.stream_feedback
                    and fake.publish_sport
                    and fake.sport_callback is not None
                    and fake.move_code == 0
                ):
                    fake.sport_callback(
                        SimpleNamespace(
                            velocity=[vx + 0.05, vy - 0.02, 0.0], yaw_speed=vyaw + 0.03
                        )
                    )
                return fake.move_code

        class ChannelFactory:
            __initialized = False
            __qos = None

            def Init(self, domain, interface, qos=None):
                cls = type(self)
                if not cls.__initialized:
                    fake.record(("dds", domain, interface))
                    cls.__qos = qos
                    cls.__initialized = True
                return True

        def initialize(domain: int, interface: str) -> None:
            ChannelFactory().Init(domain, interface)

        class CRC:
            def Crc(self, message):
                return 42

        class Publisher:
            def __init__(self, topic, dtype):
                fake.record(("publisher", topic))

            def Init(self):
                fake.record(("publisher_init",))

            def Write(self, message, timeout=None):
                fake.record(("lowcmd", [m.tau for m in message.motor_cmd], message.crc))
                return fake.lowcmd_result

            def Close(self):
                fake.record(("publisher_close",))

        class MotionSwitcher:
            def SetTimeout(self, timeout):
                pass

            def Init(self):
                pass

            def CheckMode(self):
                return 0, {"name": fake.motion_mode}

        def lowcmd():
            return SimpleNamespace(
                head=[0, 0],
                level_flag=0,
                gpio=0,
                crc=0,
                motor_cmd=[
                    SimpleNamespace(mode=0, q=0, dq=0, kp=0, kd=0, tau=0)
                    for _ in range(20)
                ],
            )

        definitions = {
            "unitree_sdk2py.core.channel": {
                "ChannelFactory": ChannelFactory,
                "ChannelFactoryInitialize": initialize,
                "Qos": object,
                "ChannelSubscriber": Subscriber,
                "ChannelPublisher": Publisher,
            },
            "unitree_sdk2py.idl.default": {"unitree_go_msg_dds__LowCmd_": lowcmd},
            "unitree_sdk2py.comm.motion_switcher.motion_switcher_client": {
                "MotionSwitcherClient": MotionSwitcher
            },
            "unitree_sdk2py.utils.crc": {"CRC": CRC},
            "unitree_sdk2py.go2.sport.sport_client": {"SportClient": Client},
            "unitree_sdk2py.idl.unitree_go.msg.dds_": {
                "LowState_": SimpleNamespace,
                "LowCmd_": SimpleNamespace,
                "SportModeState_": SimpleNamespace,
            },
        }
        modules = {}
        for name, values in definitions.items():
            module = ModuleType(name)
            module.__dict__.update(values)
            modules[name] = module
            while "." in name:
                parent_name, child_name = name.rsplit(".", 1)
                parent = modules.setdefault(parent_name, ModuleType(parent_name))
                parent.__path__ = []
                setattr(parent, child_name, module)
                module, name = parent, parent_name
        modules["unitree_sdk2py"].__version__ = "mock"
        return modules
