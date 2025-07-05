# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Test files include a copyright notice."""
from pathlib import Path

import pytest
from ament_copyright.main import main


@pytest.mark.copyright
@pytest.mark.linter
def test_copyright():
    """Copyright test case."""
    ros2_dir = Path(__file__).parent.parent.parent
    assert ros2_dir.name == "ros2", "This test must be run from the ROS 2 directory"
    assert ros2_dir.parent.name == "Tools", "This test must be run in the Tools/ros2"
    rc = main(argv=[str(ros2_dir), "test"])
    assert rc == 0, "Found errors"
