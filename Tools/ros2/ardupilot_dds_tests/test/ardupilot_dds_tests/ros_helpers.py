# Copyright 2026 ArduPilot.org.
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

"""Helpers for managing the rclpy lifecycle in the AP_DDS tests."""

import threading

from contextlib import contextmanager

import rclpy

from rclpy.executors import SingleThreadedExecutor

# How long to wait for a spin thread to notice its executor was shut down.
SPIN_JOIN_TIMEOUT = 10.0


@contextmanager
def ros_node(node_class, *args, **kwargs):
    """
    Initialise rclpy, construct and spin a node, then tear it all down again.

    Spinning on a private executor, rather than via rclpy.spin(), is what lets
    the spin thread be stopped and joined and the node destroyed while the
    context is still valid.

    Keep the executor in a local: assigning it to ``node.executor`` would not
    hold it, as that property's setter keeps only a weak reference.
    """
    rclpy.init()
    node = None
    executor = None
    spin_thread = None
    body_succeeded = False
    try:
        node = node_class(*args, **kwargs)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        spin_thread = threading.Thread(target=executor.spin)
        spin_thread.start()
        yield node
        body_succeeded = True
    finally:
        # Stop the executor first: that is what makes spin() return, so the
        # thread must be joined before the node may be destroyed.  Bound the
        # wait for in-flight callbacks so a stuck one fails the test rather
        # than hanging it.
        if executor is not None:
            executor.shutdown(timeout_sec=SPIN_JOIN_TIMEOUT)
        if spin_thread is not None:
            spin_thread.join(timeout=SPIN_JOIN_TIMEOUT)
            # Only complain about a stuck spin thread when the test itself
            # passed, so this cannot mask the real failure.
            if body_succeeded:
                assert not spin_thread.is_alive(), "Spin thread did not stop."
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
