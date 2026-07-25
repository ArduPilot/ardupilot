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

import collections
try:
    # noinspection PyUnresolvedReferences
    collections_abc = collections.abc
except AttributeError:
    collections_abc = collections


logger = getLogger(__name__)


class MessageCollector(collections_abc.Mapping):
    """This class keeps the latest TransferEvent of a given message type categorized by specified key.
    The stored items can be automatically removed if they were not updated in a specified time interval.

    Defaults are as follows:
     - Categorization key: source node ID
     - Entry timeout: infinite
    """

    def __init__(self, node, data_type, key=None, timeout=None):
        """
        :param node: Node instance.

        :param data_type: Data type to subscribe to.

        :param key: A callable that accepts a TransferEvent instance and returns a hashable.
                    The returned hashable will be used as categorization key.
                    If this argument is not provided, the messages will be categorized by source node ID.

        :param timeout: Entry timeout. If an entry was not updated in this time, it will be removed.
                        By default entry lifetime is not limited.
        """
        self._handle = node.add_handler(data_type, lambda e: self._storage.update({self._key_function(e): e}))
        self._storage = {}
        self._key_function = key or (lambda e: e.transfer.source_node_id)
        self._timeout = timeout

    def close(self):
        self._handle.remove()

    def __getitem__(self, key):
        if self._timeout is not None:
            if (self._storage[key].transfer.ts_monotonic + self._timeout) < time.monotonic():
                del self._storage[key]
        return self._storage[key]

    def __iter__(self):
        for x in list(self._storage.keys())[:]:
            try:
                self[x]         # __getitem__ is mutating here - it removes outdated entries
            except KeyError:
                pass
        return iter(self._storage)

    def __len__(self):
        return len(self._storage)
