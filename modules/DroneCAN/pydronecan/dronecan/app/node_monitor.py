#
# Copyright (C) 2014-2015  UAVCAN Development Team  <uavcan.org>
#
# This software is distributed under the terms of the MIT License.
#
# Author: Ben Dyer <ben_dyer@mac.com>
#         Pavel Kirienko <pavel.kirienko@zubax.com>
#

from __future__ import division, absolute_import, print_function, unicode_literals
import time
from logging import getLogger
import dronecan
from dronecan import uavcan

logger = getLogger(__name__)


class NodeMonitor(object):
    TIMEOUT = uavcan.protocol.NodeStatus().OFFLINE_TIMEOUT_MS / 1000  # @UndefinedVariable
    TRANSFER_PRIORITY = dronecan.TRANSFER_PRIORITY_LOWEST - 1
    MIN_RETRY_INTERVAL = 0.5
    MAX_RETRIES = 10

    class Entry:
        def __init__(self):
            self.node_id = None
            self.status = None
            self.info = None
            self.monotonic_timestamp = None
            self._remaining_retries = NodeMonitor.MAX_RETRIES
            self._info_requested_at = 0

        @property
        def discovered(self):
            return self.info is not None or self._remaining_retries <= 0

        def _update_from_status(self, e):
            self.monotonic_timestamp = e.transfer.ts_monotonic
            self.node_id = e.transfer.source_node_id
            if self.status and e.message.uptime_sec < self.status.uptime_sec:
                self._remaining_retries = NodeMonitor.MAX_RETRIES
                self.info = None
            self.status = e.message
            if self.info:
                self.info.status = self.status

        def _update_from_info(self, e):
            self._remaining_retries = NodeMonitor.MAX_RETRIES
            self.monotonic_timestamp = e.transfer.ts_monotonic
            self.node_id = e.transfer.source_node_id
            self.status = e.response.status
            self.info = e.response

        def _register_retry(self):
            assert self._remaining_retries > 0
            self._remaining_retries -= 1

        def __str__(self):
            return '%d:%s' % (self.node_id, self.info if self.info else self.status)

        __repr__ = __str__

    class UpdateEvent:
        EVENT_ID_NEW = 'new'
        EVENT_ID_INFO_UPDATE = 'info_update'
        EVENT_ID_OFFLINE = 'offline'

        def __init__(self, entry, event_id):
            self.entry = entry
            self.event_id = event_id

        def __str__(self):
            return self.event_id + ':' + str(self.entry)

        __repr__ = __str__

    class UpdateHandlerRemover:
        def __init__(self, remover):
            self._remover = remover

        def remove(self):
            self._remover()

        def try_remove(self):
            try:
                self._remover()
            except ValueError:
                pass

    def __init__(self, node):
        self._update_callbacks = []
        self._handle = node.add_handler(uavcan.protocol.NodeStatus, self._on_node_status)  # @UndefinedVariable
        self._info_handle = node.add_handler(uavcan.protocol.GetNodeInfo, self._on_info_response, sniff_response=True)  # @UndefinedVariable
        self._registry = {}  # {node_id: Entry}
        self._timer = node.periodic(1, self._remove_stale)

    def add_update_handler(self, callback):
        """
        Args:
            callback:   The specified callback will be invoked when:
                        - A new node appears
                        - Node info for an existing node gets updated
                        - Node goes offline
        Returns:    Call remove() or try_remove() on the returned object to unregister the handler.
        """
        self._update_callbacks.append(callback)
        return self.UpdateHandlerRemover(lambda: self._update_callbacks.remove(callback))

    def _call_event_handlers(self, event):
        for cb in self._update_callbacks:
            cb(event)

    def exists(self, node_id):
        """
        Args:
            node_id:    Returns True if the given node ID exists, false otherwise
        """
        return node_id in self._registry

    def get(self, node_id):
        """
        Args:
            node_id:    Returns an Entry instance for the given node ID.
                        If the requested node ID does not exist, throws KeyError.
        """
        if (self._registry[node_id].monotonic_timestamp + self.TIMEOUT) < time.monotonic():
            self._call_event_handlers(self.UpdateEvent(self._registry[node_id],
                                                       self.UpdateEvent.EVENT_ID_OFFLINE))
            del self._registry[node_id]
        return self._registry[node_id]

    def get_all_node_id(self):
        """Returns a generator or an iterable containing all currently active node ID."""
        return self._registry.keys()

    def find_all(self, predicate):
        """Returns a generator that produces a sequence of Entry objects for which the predicate returned True.
        Args:
            predicate:  A callable that returns a value coercible to bool.
        """
        for _nid, entry in self._registry.items():
            if predicate(entry):
                yield entry

    def are_all_nodes_discovered(self):
        """Reports whether there are nodes whose node info is still unknown."""
        undiscovered = self.find_all(lambda e: not e.discovered)
        return len(list(undiscovered)) == 0

    def close(self):
        """Stops the instance. The registry will not be cleared."""
        self._handle.remove()
        self._timer.remove()

    def _remove_stale(self):
        for nid, e in list(self._registry.items())[:]:
            if (e.monotonic_timestamp + self.TIMEOUT) < time.monotonic():
                del self._registry[nid]
                self._call_event_handlers(self.UpdateEvent(e, self.UpdateEvent.EVENT_ID_OFFLINE))

    def _on_node_status(self, e):
        node_id = e.transfer.source_node_id

        try:
            entry = self.get(node_id)
            new_entry = False
        except KeyError:
            entry = self.Entry()
            self._registry[node_id] = entry
            new_entry = True

        # noinspection PyProtectedMember
        entry._update_from_status(e)
        if new_entry:
            self._call_event_handlers(self.UpdateEvent(entry, self.UpdateEvent.EVENT_ID_NEW))

        if not entry.discovered and not e.node.is_anonymous:
            should_retry_now = entry.monotonic_timestamp - entry._info_requested_at > self.MIN_RETRY_INTERVAL
            if should_retry_now:
                entry._info_requested_at = entry.monotonic_timestamp
                # noinspection PyProtectedMember
                entry._register_retry()
                e.node.request(uavcan.protocol.GetNodeInfo.Request(), node_id,  # @UndefinedVariable
                               priority=self.TRANSFER_PRIORITY, callback=self._on_info_response)

    def _on_info_response(self, e):
        if not e:
            return

        try:
            entry = self.get(e.transfer.source_node_id)
        except KeyError:
            entry = self.Entry()
            self._registry[e.transfer.source_node_id] = entry

        # noinspection PyProtectedMember
        entry._update_from_info(e)

        hw_unique_id = "".join(format(c, "02X") for c in e.response.hardware_version.unique_id)
        msg = (
            "[#{0:03d}:uavcan.protocol.GetNodeInfo] " +
            "software_version.major={1:d} " +
            "software_version.minor={2:d} " +
            "software_version.vcs_commit={3:08x} " +
            "software_version.image_crc={4:016X} " +
            "hardware_version.major={5:d} " +
            "hardware_version.minor={6:d} " +
            "hardware_version.unique_id={7!s} " +
            "name={8!r}"
        ).format(
            e.transfer.source_node_id,
            e.response.software_version.major,
            e.response.software_version.minor,
            e.response.software_version.vcs_commit,
            e.response.software_version.image_crc,
            e.response.hardware_version.major,
            e.response.hardware_version.minor,
            hw_unique_id,
            e.response.name.decode()
        )
        logger.info(msg)

        self._call_event_handlers(self.UpdateEvent(entry, self.UpdateEvent.EVENT_ID_INFO_UPDATE))
