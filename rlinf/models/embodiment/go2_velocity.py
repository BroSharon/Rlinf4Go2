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

"""Fixed body velocity commands for evaluation of the Go2 Sport controller."""

import math
from typing import Any, NoReturn

import torch
from omegaconf import DictConfig

from rlinf.models.embodiment.base_policy import BasePolicy


class Go2VelocityPolicy(torch.nn.Module, BasePolicy):
    """Repeat one signed body-axis command for each state in the batch."""

    _DIRECTIONS = {
        "forward": (1, 0, 0),
        "backward": (-1, 0, 0),
        "left": (0, 1, 0),
        "right": (0, -1, 0),
        "turn_left": (0, 0, 1),
        "turn_right": (0, 0, -1),
    }

    def __init__(self, cfg: DictConfig) -> None:
        super().__init__()
        if (cfg.num_action_chunks, cfg.action_dim) != (1, 3):
            raise ValueError(
                "Go2 velocity evaluation requires a single three-axis action, "
                f"got ({cfg.num_action_chunks}, {cfg.action_dim})."
            )
        if cfg.command_name not in self._DIRECTIONS:
            raise ValueError(
                f"Expected a Go2 direction from {tuple(self._DIRECTIONS)}, "
                f"got {cfg.command_name!r}."
            )
        magnitude = cfg.command_magnitude
        if not math.isfinite(magnitude) or magnitude <= 0:
            raise ValueError(
                f"Expected a positive finite command magnitude, got {magnitude}."
            )
        direction = torch.tensor(
            self._DIRECTIONS[cfg.command_name], dtype=torch.float32
        )
        self.register_buffer("velocity", magnitude * direction)

    def predict_action_batch(
        self, env_obs: dict[str, torch.Tensor], **kwargs: Any
    ) -> tuple[torch.Tensor, dict[str, Any]]:
        """Return one [vx, vy, yaw_rate] command per environment and control step."""
        return self.velocity.expand(len(env_obs["states"]), 1, 3).contiguous(), {}

    def default_forward(self, **kwargs: Any) -> NoReturn:
        """Training is undefined for this fixed evaluation command."""
        raise NotImplementedError("Go2 velocity commands are for evaluation only.")
