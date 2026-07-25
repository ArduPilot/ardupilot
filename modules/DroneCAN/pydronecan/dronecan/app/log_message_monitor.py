#
# Copyright (C) 2014-2015  UAVCAN Development Team  <uavcan.org>
#
# This software is distributed under the terms of the MIT License.
#
# Author: Ben Dyer <ben_dyer@mac.com>
#         Pavel Kirienko <pavel.kirienko@zubax.com>
#

from __future__ import division, absolute_import, print_function, unicode_literals
from logging import getLogger
from dronecan import uavcan

logger = getLogger(__name__)


class LogMessageMonitor(object):
    def __init__(self, node):
        self._handle = node.add_handler(uavcan.protocol.debug.LogMessage, self._on_message)  # @UndefinedVariable

    def close(self):
        self._handle.remove()

    def _on_message(self, e):
        logmsg = "LogMessageMonitor [#{0:03d}:{1}] {2}"\
            .format(e.transfer.source_node_id, e.message.source.decode(), e.message.text.decode())
        (logger.debug, logger.info, logger.warning, logger.error)[e.message.level.value](logmsg)
