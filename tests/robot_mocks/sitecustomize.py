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

"""Install fake SDKs at worker startup when ``RLINF_ROBOT_MOCKS`` is set."""

import os

if os.environ.get("RLINF_ROBOT_MOCKS"):
    try:
        import sys

        if os.environ["RLINF_ROBOT_MOCKS"] == "go2":
            from unitree_go2 import FakeGo2SDK

            fake = FakeGo2SDK(
                stream_feedback=True, trace_path=os.environ.get("RLINF_GO2_MOCK_TRACE")
            )
            sys.modules.update(fake.modules())
        else:
            from robot_mocks import sdk_modules

            sys.modules.update(sdk_modules())
    except Exception as error:  # pragma: no cover - a worker must still start
        if os.environ["RLINF_ROBOT_MOCKS"] == "go2":
            raise SystemExit(f"Go2 mock SDK bootstrap failed: {error!r}") from error
        print(f"[robot_mocks] could not install fake SDKs: {error!r}")
