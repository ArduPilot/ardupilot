# Copyright 2023 ArduPilot.org.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.

# Adapted from launch.actions.opaque_function.py

# Copyright 2018 Open Source Robotics Foundation, Inc.
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

"""Module for the OpaqueFunction action."""

import collections.abc
from typing import Any
from typing import Callable
from typing import Dict
from typing import Iterable
from typing import List
from typing import Optional
from typing import Text

from launch.action import Action
from launch.launch_context import LaunchContext
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.utilities import ensure_argument_type


class ExecuteFunction(Action):
    """
    Action that executes a Python function.

    The signature of the function should be:

    .. code-block:: python

        def function(
            context: LaunchContext,
            *args,
            **kwargs
        ) -> Optional[Action]:
            ...

    """

    def __init__(
        self,
        *,
        function: Callable,
        args: Optional[Iterable[Any]] = None,
        kwargs: Optional[Dict[Text, Any]] = None,
        **left_over_kwargs
    ) -> None:
        """Create an ExecuteFunction action."""
        super().__init__(**left_over_kwargs)
        if not callable(function):
            raise TypeError("ExecuteFunction expected a callable for 'function', got '{}'".format(type(function)))
        ensure_argument_type(args, (collections.abc.Iterable, type(None)), "args", "ExecuteFunction")
        ensure_argument_type(kwargs, (dict, type(None)), "kwargs", "ExecuteFunction")
        self.__function = function
        self.__args = []  # type: Iterable
        if args is not None:
            self.__args = args
        self.__kwargs = {}  # type: Dict[Text, Any]
        if kwargs is not None:
            self.__kwargs = kwargs

        self.__action = None

    @property
    def action(self) -> Action:
        """Return the Action obtained by executing the function."""
        return self.__action

    def execute(self, context: LaunchContext) -> Optional[List[LaunchDescriptionEntity]]:
        """Execute the function."""
        action = self.__function(context, *self.__args, **self.__kwargs)
        self.__action = action
        return [action]
