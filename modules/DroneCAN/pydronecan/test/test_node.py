#
# Copyright (C) 2014-2015  UAVCAN Development Team  <uavcan.org>
#
# This software is distributed under the terms of the MIT License.
#
# Author: Ben Dyer <ben_dyer@mac.com>
#         Pavel Kirienko <pavel.kirienko@zubax.com>
#

import unittest
try:
    from unittest.mock import Mock, ANY
except ImportError:
    from mock import Mock, ANY

import dronecan
from dronecan.node import Node
import dronecan.transport as transport
from dronecan.driver.common import CANFrame

import os.path

DEFAULT_TRANSFER_PRIORITY = 20

class TestException(RuntimeError):
    pass


class HandlerExceptionHandling(unittest.TestCase):
    def setUp(self):
        dronecan.load_dsdl()
        msg = dronecan.uavcan.protocol.debug.KeyValue()
        msg.key = 'foo'
        msg.value = 42

        transfer = transport.Transfer(
            payload=msg,
            source_node_id=42,
            transfer_id=10,
            transfer_priority=DEFAULT_TRANSFER_PRIORITY,
            service_not_message=False)

        self.frames = [
            CANFrame(can_id=f.message_id, data=f.bytes, extended=True)
            for f in transfer.to_frames()
        ]
        self.driver = Mock()
        self.driver.receive.side_effect = self.frames + [None]

    def _raise_callback(self, event):
        raise TestException("test exception")

    def _spin(self):
        try:
            self.node.spin()
        except StopIteration:  # due to using mocks for can frames
            pass

    def test_exceptions_are_caught_by_spin(self):
        self.node = Node(self.driver)
        self.node.add_handler(dronecan.uavcan.protocol.debug.KeyValue,
                              self._raise_callback)

        self._spin()

    def test_exceptions_are_not_caught_by_spin(self):
        self.node = Node(self.driver, catch_handler_exceptions=False)
        self.node.add_handler(dronecan.uavcan.protocol.debug.KeyValue,
                              self._raise_callback)

        with self.assertRaises(RuntimeError):
            self._spin()

    def test_second_handler_is_called_after_first_one_raises(self):
        mock_cb = Mock(return_value=None)
        self.node = Node(self.driver)
        self.node.add_handler(dronecan.uavcan.protocol.debug.KeyValue,
                              self._raise_callback)
        self.node.add_handler(dronecan.uavcan.protocol.debug.KeyValue,
                              mock_cb)

        self._spin()
        mock_cb.assert_any_call(ANY)



if __name__ == '__main__':
    unittest.main()
