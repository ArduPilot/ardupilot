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

"""Utilities."""
from functools import wraps
from typing import List

from launch import LaunchContext
from launch import LaunchDescriptionEntity


# Adapted from a SO answer by David Wolever.
# Is there a library function in Python to turn a generator-function
# into a function returning a list?
# https://stackoverflow.com/questions/12377013
def listify(fn) -> List[LaunchDescriptionEntity]:
    """Wrap a functions's return value in a list."""

    @wraps(fn)
    def listify_helper(context: LaunchContext, *args, **kwargs) -> List[LaunchDescriptionEntity]:
        return [fn(context, args, kwargs)]

    return listify_helper
