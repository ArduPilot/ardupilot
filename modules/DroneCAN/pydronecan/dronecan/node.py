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
import collections
import sched
import sys
import inspect
from logging import getLogger

import dronecan
import dronecan.driver as driver
import dronecan.transport as transport
from dronecan.transport import get_dronecan_data_type
from dronecan import UAVCANException


DEFAULT_NODE_STATUS_INTERVAL = 1.0
DEFAULT_SERVICE_TIMEOUT = 1.0
DEFAULT_TRANSFER_PRIORITY = 20


logger = getLogger(__name__)


class Scheduler(object):
    """This class implements a simple non-blocking event scheduler.
    It supports one-shot and periodic events.
    """

    def __init__(self):
        if sys.version_info[0] > 2:
            # Nice and easy.
            self._scheduler = sched.scheduler()
            # The documentation says that run() returns the next deadline,
            # but it's not true - it returns the remaining time.
            self._run_scheduler = lambda: self._scheduler.run(blocking=False) + self._scheduler.timefunc()
        else:
            # Nightmare inducing hacks
            class SayNoToBlockingSchedulingException(dronecan.UAVCANException):
                pass

            def delayfunc_impostor(duration):
                if duration > 0:
                    raise SayNoToBlockingSchedulingException('No!')

            self._scheduler = sched.scheduler(time.monotonic, delayfunc_impostor)

            def run_scheduler():
                try:
                    self._scheduler.run()
                except SayNoToBlockingSchedulingException:
                    q = self._scheduler.queue
                    return q[0][0] if q else None

            self._run_scheduler = run_scheduler

    def _make_sched_handle(self, get_event):
        class EventHandle(object):
            @staticmethod
            def remove():
                self._scheduler.cancel(get_event())

            @staticmethod
            def try_remove():
                try:
                    self._scheduler.cancel(get_event())
                    return True
                except ValueError:
                    return False

        return EventHandle()

    def _poll_scheduler_and_get_next_deadline(self):
        return self._run_scheduler()

    def defer(self, timeout_seconds, callback):
        """This method allows to invoke the callback with specified arguments once the specified amount of time.
        :returns: EventHandle object. Call .remove() on it to cancel the event.
        """
        priority = 1
        event = self._scheduler.enter(timeout_seconds, priority, callback, ())
        return self._make_sched_handle(lambda: event)

    def periodic(self, period_seconds, callback):
        """This method allows to invoke the callback periodically, with specified time intervals.
        Note that the scheduler features zero phase drift.
        :returns: EventHandle object. Call .remove() on it to cancel the event.
        """
        priority = 0

        def caller(scheduled_deadline):
            # Event MUST be re-registered first in order to ensure that it can be cancelled from the callback
            scheduled_deadline += period_seconds
            event_holder[0] = self._scheduler.enterabs(scheduled_deadline, priority, caller, (scheduled_deadline,))
            callback()

        first_deadline = self._scheduler.timefunc() + period_seconds
        event_holder = [self._scheduler.enterabs(first_deadline, priority, caller, (first_deadline,))]
        return self._make_sched_handle(lambda: event_holder[0])

    def has_pending_events(self):
        """Returns true if there is at least one pending event in the queue.
        """
        return not self._scheduler.empty()


class TransferEvent(object):
    def __init__(self, transfer, node, payload_attr_name):
        setattr(self, payload_attr_name, transfer.payload)
        self.transfer = transfer
        self.node = node

    def __str__(self):
        return str(self.transfer)

    def __repr__(self):
        return repr(self.transfer)


class HandleRemover:
    def __init__(self, remover):
        self._remover = remover

    def remove(self):
        self._remover()

    def try_remove(self):
        try:
            self._remover()
            return True
        except ValueError:
            return False


class HandlerDispatcher(object):
    def __init__(self, node, catch_exceptions):
        self._handlers = []  # type, callable
        self._node = node
        self._catch_exceptions = catch_exceptions

    def add_handler(self, dronecan_type, handler, sniff_response=False, **kwargs):
        service = False
        if dronecan_type is not None and dronecan_type.kind == dronecan_type.KIND_SERVICE:
            service = True
        sniff_response = sniff_response and service
        logger.info('Adding handler for %s %s', 'service' if service else 'message', dronecan_type)
        # If handler is a class, create a wrapper function and register it as a regular callback
        if inspect.isclass(handler):
            def class_handler_adapter(event):
                h = handler(event, **kwargs)
                if service and not sniff_response:
                    h.on_request()
                    return h.response
                else:
                    h.on_message()

            return self.add_handler(dronecan_type, class_handler_adapter, sniff_response)

        # At this point process the handler as a regular callback
        def call(transfer):
            msgtype = 'response' if sniff_response else 'request'
            event = TransferEvent(transfer, self._node, msgtype if service else 'message')
            result = handler(event, **kwargs)
            if service and not sniff_response:
                if not transfer.request_not_response:
                    return
                if result is None:
                    raise UAVCANException('Service request handler did not return a response [%r, %r]' %
                                          (dronecan_type, handler))
                self._node.respond(result,
                                   transfer.source_node_id,
                                   transfer.transfer_id,
                                   transfer.transfer_priority)
            else:
                if result is not None:
                    raise UAVCANException('Message request handler did not return None [%r, %r]' %
                                          (dronecan_type, handler))

        entry = dronecan_type, call, sniff_response
        self._handlers.append(entry)
        return HandleRemover(lambda: self._handlers.remove(entry))

    def remove_handlers(self, dronecan_type):
        self._handlers = list(filter(lambda x: x[0] != dronecan_type, self._handlers))

    def call_handlers(self, transfer):
        for dronecan_type, wrapper, _ in self._handlers:
            if dronecan_type is None or dronecan_type == get_dronecan_data_type(transfer.payload):
                try:
                    wrapper(transfer)
                # noinspection PyBroadException
                except Exception as e:
                    logger.error('Transfer handler exception', exc_info=True)
                    if not self._catch_exceptions:
                        raise e

    def is_response_sniffing_enabled(self, transfer):
        if not transfer.service_not_message:
            return False
        if transfer.request_not_response:
            return False
        dronecan_type = get_dronecan_data_type(transfer.payload)
        for t, _, sniff_response in self._handlers:
            if t == dronecan_type and sniff_response:
                return True
        return False

class TransferHookDispatcher(object):
    TRANSFER_DIRECTION_INCOMING = 'rx'
    TRANSFER_DIRECTION_OUTGOING = 'tx'

    def __init__(self):
        self._hooks = []

    def add_hook(self, hook, **kwargs):
        def proxy(transfer):
            hook(transfer, **kwargs)
        self._hooks.append(proxy)
        return HandleRemover(lambda: self._hooks.remove(proxy))

    def call_hooks(self, direction, transfer):
        setattr(transfer, 'direction', direction)
        for hook in self._hooks:
            # noinspection PyBroadException
            try:
                hook(transfer)
            except Exception:
                logger.error('Transfer hook exception', exc_info=True)


class Node(Scheduler):
    def __init__(self, can_driver, node_id=None, node_status_interval=None,
                 mode=None, node_info=None, catch_handler_exceptions=True,
                 send_canfd=False,
                 **_extras):
        """
        It is recommended to use make_node() rather than instantiating this type directly.

        :param can_driver: CAN bus driver object. Calling close() on a node object closes its driver instance.

        :param node_id: Node ID of the current instance. Defaults to None, which enables passive mode.

        :param node_status_interval: NodeStatus broadcasting interval. Defaults to DEFAULT_NODE_STATUS_INTERVAL.

        :param mode: Initial operating mode (INITIALIZATION, OPERATIONAL, etc.); defaults to INITIALIZATION.

        :param node_info: Structure of type uavcan.protocol.GetNodeInfo.Response, responded with when the local
                          node is queried for its node info.
        :param catch_handler_exceptions: If true, exceptions raised from message
                                         handlers will be caught and logged. If
                                         False, spin() will raise exceptions
                                         raised from callbacks.
        """
        super(Node, self).__init__()

        self._handler_dispatcher = HandlerDispatcher(self, catch_handler_exceptions)

        self._can_driver = can_driver
        self._node_id = node_id
        self._send_canfd = send_canfd

        self._transfer_manager = transport.TransferManager()
        self._outstanding_requests = {}
        self._outstanding_request_callbacks = {}
        self._next_transfer_ids = collections.defaultdict(int)

        self.start_time_monotonic = time.monotonic()

        # Hooks
        self._transfer_hook_dispatcher = TransferHookDispatcher()

        # NodeStatus publisher
        self.health = dronecan.uavcan.protocol.NodeStatus().HEALTH_OK                                    # @UndefinedVariable
        self.mode = dronecan.uavcan.protocol.NodeStatus().MODE_INITIALIZATION if mode is None else mode  # @UndefinedVariable
        self.vendor_specific_status_code = 0

        node_status_interval = node_status_interval or DEFAULT_NODE_STATUS_INTERVAL
        self.periodic(node_status_interval, self._send_node_status)

        # GetNodeInfo server
        def on_get_node_info(e):
            logger.debug('GetNodeInfo request from %r', e.transfer.source_node_id)
            self._fill_node_status(self.node_info.status)
            return self.node_info
        self.node_info = node_info or dronecan.uavcan.protocol.GetNodeInfo.Response()     # @UndefinedVariable
        self.add_handler(dronecan.uavcan.protocol.GetNodeInfo, on_get_node_info)          # @UndefinedVariable

    def set_canfd(self, send_canfd):
        '''change default send of CANFD'''
        self._send_canfd = send_canfd

    @property
    def is_anonymous(self):
        return (self._node_id or 0) == 0

    @property
    def node_id(self):
        return self._node_id

    @node_id.setter
    def node_id(self, value):
        if self.is_anonymous:
            value = int(value)
            if not (1 <= value <= 127):
                raise ValueError('Invalid Node ID [%d]' % value)
            self._node_id = value
        else:
            raise UAVCANException('Node ID can be set only once')

    @property
    def can_driver(self):
        return self._can_driver

    def _recv_frame(self, raw_frame):
        if not raw_frame.extended:
            return

        frame = transport.Frame(raw_frame.id, raw_frame.data, raw_frame.ts_monotonic, raw_frame.ts_real, raw_frame.canfd)

        transfer_frames = self._transfer_manager.receive_frame(frame)
        if not transfer_frames:
            return

        transfer = transport.Transfer()
        transfer.from_frames(transfer_frames)

        self._transfer_hook_dispatcher.call_hooks(self._transfer_hook_dispatcher.TRANSFER_DIRECTION_INCOMING, transfer)

        if (transfer.service_not_message and not transfer.request_not_response) and \
                transfer.dest_node_id == self._node_id:
            # This is a reply to a request we sent. Look up the original request and call the appropriate callback
            requests = self._outstanding_requests.keys()
            for key in requests:
                if transfer.is_response_to(self._outstanding_requests[key]):
                    # Call the request's callback and remove it from the active list
                    event = TransferEvent(transfer, self, 'response')
                    self._outstanding_request_callbacks[key](event)
                    del self._outstanding_requests[key]
                    del self._outstanding_request_callbacks[key]
                    break
        elif not transfer.service_not_message or \
            self._handler_dispatcher.is_response_sniffing_enabled(transfer) or \
            transfer.dest_node_id == self._node_id:
            # This is a request or a broadcast; look up the appropriate handler by data type ID
            self._handler_dispatcher.call_handlers(transfer)

    def _next_transfer_id(self, key):
        transfer_id = self._next_transfer_ids[key]
        self._next_transfer_ids[key] = (transfer_id + 1) & 0x1F
        return transfer_id

    def _throw_if_anonymous(self):
        if not self._node_id:
            raise dronecan.UAVCANException('The local node is configured in anonymous mode')

    def _fill_node_status(self, msg):
        msg.uptime_sec = int(time.monotonic() - self.start_time_monotonic + 0.5)
        msg.health = self.health
        msg.mode = self.mode
        msg.vendor_specific_status_code = self.vendor_specific_status_code

    def _send_node_status(self):
        self._fill_node_status(self.node_info.status)
        if self._node_id:
            # TODO: transmit self.node_info.status instead of creating a new object
            msg = dronecan.uavcan.protocol.NodeStatus()  # @UndefinedVariable
            self._fill_node_status(msg)
            self.broadcast(msg)

    def add_transfer_hook(self, hook, **kwargs):
        """
        :param hook:    Callable hook; expected signature: hook(transfer).
        :param kwargs:  Extra arguments for the hook.
        :return:        A handle object that can be used to unregister the hook by calling remove() on it.
        """
        return self._transfer_hook_dispatcher.add_hook(hook, **kwargs)

    def add_handler(self, dronecan_type, handler, **kwargs):
        """
        Adds a handler for the specified data type.
        :param dronecan_type: DSDL data type. Only transfers of this type will be accepted for this handler.
        :param handler:     The handler. This must be either a callable or a class.
        :param **kwargs:    Extra arguments for the handler.
        :return: A remover object that can be used to unregister the handler as follows:
            x = node.add_handler(...)
            # Remove the handler like this:
            x.remove()
            # Or like this:
            if x.try_remove():
                print('The handler has been removed successfully')
            else:
                print('There is no such handler')
        """
        return self._handler_dispatcher.add_handler(dronecan_type, handler, **kwargs)

    def remove_handlers(self, dronecan_type):
        """Removes all handlers for the specified DSDL data type.
        """
        self._handler_dispatcher.remove_handlers(dronecan_type)

    def spin(self, timeout=None):
        """
        Runs background processes until timeout expires.
        Note that all processing is implemented in one thread.
        :param timeout: The method will return once this amount of time expires.
                        If None, the method will never return.
                        If zero, the method will handle only those events that are ready, then return immediately.
        :returns: number of received frames
        """
        count = 0
        if timeout != 0:
            deadline = (time.monotonic() + timeout) if timeout is not None else sys.float_info.max

            def execute_once():
                next_event_at = self._poll_scheduler_and_get_next_deadline()
                if next_event_at is None:
                    next_event_at = sys.float_info.max

                read_timeout = min(next_event_at, deadline) - time.monotonic()
                read_timeout = max(read_timeout, 0)
                read_timeout = min(read_timeout, 1)

                frame = self._can_driver.receive(read_timeout)
                if frame:
                    self._recv_frame(frame)
                    return 1
                return 0

            count += execute_once()
            while time.monotonic() < deadline:
                execute_once()
        else:
            while True:
                frame = self._can_driver.receive(0)
                if frame:
                    self._recv_frame(frame)
                    count += 1
                else:
                    break
            self._poll_scheduler_and_get_next_deadline()
        return count

    def request(self, payload, dest_node_id, callback, priority=None, timeout=None, canfd=None):
        self._throw_if_anonymous()

        if canfd is None:
            canfd = self._send_canfd

        # Preparing the transfer
        transfer_id = self._next_transfer_id((get_dronecan_data_type(payload).default_dtid, dest_node_id))
        transfer = transport.Transfer(payload=payload,
                                      source_node_id=self._node_id,
                                      dest_node_id=dest_node_id,
                                      transfer_id=transfer_id,
                                      transfer_priority=priority or DEFAULT_TRANSFER_PRIORITY,
                                      service_not_message=True,
                                      request_not_response=True,
                                      canfd=canfd)

        # Calling hooks
        self._transfer_hook_dispatcher.call_hooks(self._transfer_hook_dispatcher.TRANSFER_DIRECTION_OUTGOING, transfer)

        # Sending the transfer
        for frame in transfer.to_frames():
            self._can_driver.send(frame.message_id, frame.bytes, extended=True, canfd=canfd)

        # Registering a callback that will be invoked if there was no response after 'timeout' seconds
        def on_timeout():
            try:
               del self._outstanding_requests[transfer.key]
            except KeyError:
                pass
            try:
                del self._outstanding_request_callbacks[transfer.key]
            except KeyError:
                pass
            callback(None)

        timeout = timeout or DEFAULT_SERVICE_TIMEOUT
        timeout_caller_handle = self.defer(timeout, on_timeout)

        # This wrapper will automatically cancel the timeout callback if there was a response
        def timeout_cancelling_wrapper(event):
            timeout_caller_handle.try_remove()
            callback(event)

        # Registering the pending request using the wrapper above instead of the callback
        self._outstanding_requests[transfer.key] = transfer
        self._outstanding_request_callbacks[transfer.key] = timeout_cancelling_wrapper

        logger.debug("Node.request(dest_node_id={0:d}): sent {1!r}".format(dest_node_id, payload))

    def respond(self, payload, dest_node_id, transfer_id, priority, canfd=None):
        self._throw_if_anonymous()

        if canfd is None:
            canfd = self._send_canfd

        transfer = transport.Transfer(
            payload=payload,
            source_node_id=self._node_id,
            dest_node_id=dest_node_id,
            transfer_id=transfer_id,
            transfer_priority=priority,
            service_not_message=True,
            request_not_response=False,
            canfd=canfd
        )

        self._transfer_hook_dispatcher.call_hooks(self._transfer_hook_dispatcher.TRANSFER_DIRECTION_OUTGOING, transfer)

        for frame in transfer.to_frames():
            self._can_driver.send(frame.message_id, frame.bytes, extended=True, canfd=canfd)

        logger.debug("Node.respond(dest_node_id={0:d}, transfer_id={0:d}, priority={0:d}): sent {1!r}"
                     .format(dest_node_id, transfer_id, priority, payload))

    def broadcast(self, payload, priority=None, canfd=None):
        self._throw_if_anonymous()

        if canfd is None:
            canfd = self._send_canfd
        
        transfer_id = self._next_transfer_id(get_dronecan_data_type(payload).default_dtid)
        transfer = transport.Transfer(payload=payload,
                                      source_node_id=self._node_id,
                                      transfer_id=transfer_id,
                                      transfer_priority=priority or DEFAULT_TRANSFER_PRIORITY,
                                      service_not_message=False,
                                      canfd=canfd)

        self._transfer_hook_dispatcher.call_hooks(self._transfer_hook_dispatcher.TRANSFER_DIRECTION_OUTGOING, transfer)

        for frame in transfer.to_frames():
            self._can_driver.send(frame.message_id, frame.bytes, extended=True, canfd=canfd)

    def close(self):
        self._can_driver.close()


def make_node(can_device_name, **kwargs):
    """Constructs a node instance with specified CAN device.
    :param can_device_name: CAN device name, e.g. "/dev/ttyACM0", "COM9", "can0".
    :param kwargs: These arguments will be supplied to the CAN driver factory and to the node constructor.
    """
    can = driver.make_driver(can_device_name, **kwargs)
    return Node(can, **kwargs)


class Monitor(object):
    def __init__(self, event):
        self.message = event.message
        self.transfer = event.transfer
        self.node = event.node

    def on_message(self):
        pass


class Service(object):
    def __init__(self, event):
        self.request = event.request
        self.transfer = event.transfer
        self.node = event.node
        self.response = get_dronecan_data_type(self.request).Response()

    def on_request(self):
        pass
