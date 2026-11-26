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

"""Exercise the public Go2 evaluator through Ray and the fake SDK2."""

import json
import os
import subprocess
from pathlib import Path

import pytest
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator


def test_go2_tracking_mock_sdk(tmp_path):
    root = Path(__file__).resolve().parents[3]
    trace = tmp_path / "sdk.jsonl"
    subprocess.run(
        [
            "bash",
            "evaluations/run_eval.sh",
            "realworld",
            "realworld_go2_tracking",
            "rollout.model.command_name=turn_left",
            "env.eval.override_cfg.is_dummy=false",
            "cluster.node_groups.0.hardware.configs.0.interface=fake-go2",
            "cluster.node_groups.0.hardware.configs.0.enable_motion=true",
            f"runner.logger.log_path={tmp_path}",
            "runner.logger.logger_backends=[tensorboard]",
        ],
        cwd=root,
        env={
            **os.environ,
            "RLINF_ROBOT_MOCKS": "go2",
            "RLINF_GO2_MOCK_TRACE": str(trace),
            "PYTHONPATH": os.pathsep.join(
                [str(root / "tests/robot_mocks"), os.environ.get("PYTHONPATH", "")]
            ),
        },
        check=True,
    )
    expected = {
        "go2_tracking_dispatched_steps": 10,
        "go2_tracking_valid_steps": 10,
        "go2_tracking_valid_episode": 1,
        "go2_tracking_feedback_coverage": 1,
        "go2_tracking_mae_vx_mps": 0.05,
        "go2_tracking_mae_vy_mps": 0.02,
        "go2_tracking_mae_yaw_rate_radps": 0.03,
    }
    log = EventAccumulator(str(tmp_path / "tensorboard")).Reload()
    for name, value in {**expected, "num_trajectories": 1, "episode_len": 10}.items():
        (scalar,) = log.Scalars(f"eval/{name}")
        assert scalar.value == pytest.approx(value)

    (evidence_path,) = tmp_path.glob("go2_tracking_*.json")
    episode = json.loads(evidence_path.read_text())
    assert episode["metrics"] == pytest.approx(expected)
    assert episode["feedback_source"] == "mock"
    assert episode["command_name"] == "turn_left"
    assert episode["end_reason"] == "max_num_steps"

    events = [json.loads(line) for line in trace.read_text().splitlines()]
    assert ["dds", 0, "fake-go2"] in events
    moves = [event for event in events if event[0] == "move"]
    assert len(moves) == len(episode["steps"]) == 10
    for move, step in zip(moves, episode["steps"]):
        assert move[1:] == pytest.approx([0, 0, 0.1])
        assert step["status"] == "valid"
        assert step["dispatch"]["command"] == move[1:]
        sample = step["feedback"]
        assert step["dispatch"]["time_ns"] <= sample["time_ns"] < step["end_ns"]
        assert sample["velocity"] == pytest.approx([0.05, -0.02, 0])
        assert sample["yaw_speed"] == pytest.approx(0.13)

    names = [event[0] for event in events]
    last_move = max(index for index, name in enumerate(names) if name == "move")
    stopped = names.index("stop", last_move + 1)
    assert names.index("close") > stopped
    assert names.index("sport_close") > stopped


def test_go2_policy_mock_sdk(tmp_path):
    import numpy as np
    import torch

    from rlinf.envs.real.go2.control import Go2Control, JointState
    from rlinf.models.embodiment.go2_policy import Go2Policy

    root = Path(__file__).resolve().parents[3]
    model = Go2Policy()
    with torch.no_grad():
        for parameter in model.parameters():
            parameter.zero_()
        model.actor[-1].bias.fill_(0.25)
        model.std.fill_(1)
    checkpoint = tmp_path / "policy.pt"
    torch.save(model.state_dict(), checkpoint)
    trace = tmp_path / "sdk.jsonl"
    result = subprocess.run(
        [
            "bash",
            "evaluations/run_eval.sh",
            "realworld",
            "realworld_go2_policy",
            f"rollout.model.model_path={checkpoint}",
            "env.eval.override_cfg.is_dummy=false",
            "env.eval.override_cfg.max_num_steps=2",
            "env.eval.max_episode_steps=2",
            "env.eval.max_steps_per_rollout_epoch=2",
            "cluster.node_groups.0.hardware.configs.0.interface=fake-go2",
            f"runner.logger.log_path={tmp_path}",
        ],
        cwd=root,
        env={
            **os.environ,
            "RLINF_ROBOT_MOCKS": "go2",
            "RLINF_GO2_MOCK_TRACE": str(trace),
            "PYTHONPATH": os.pathsep.join(
                [str(root / "tests/robot_mocks"), os.environ.get("PYTHONPATH", "")]
            ),
        },
        capture_output=True,
        text=True,
        check=True,
    )
    assert "'eval/episode_len': array(2." in result.stdout
    events = [json.loads(line) for line in trace.read_text().splitlines()]
    commands = [event for event in events if event[0] == "lowcmd"]
    assert len(commands) == 8
    state = JointState(
        q=[i / 10 for i in range(12)],
        dq=[i / 20 for i in range(12)],
        quaternion_wxyz=[1, 0, 0, 0],
        gyroscope_b=[1, 2, 3],
        timestamp=0,
    )
    expected = Go2Control("A").step(state, [0.25] * 12, 0)["torque_sdk"]
    np.testing.assert_allclose(commands[0][1][:12], expected, atol=1e-6)
    assert commands[0][2] == 42
    assert [event[0] for event in events[-2:]] == ["publisher_close", "close"]
