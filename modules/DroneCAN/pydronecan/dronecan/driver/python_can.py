#
# Copyright (C) 2019 UAVCAN Development Team  <uavcan.org>
#
# This software is distributed under the terms of the MIT License.
#
# Author: Jos Vos <jos.vos@demcon.nl>
#

from __future__ import division, absolute_import, print_function, unicode_literals
import os
import errno
import time
import threading
from logging import getLogger

from .common import DriverError, TxQueueFullError, CANFrame, AbstractDriver
from .timestamp_estimator import TimestampEstimator

try:
    import queue
except ImportError:
    # noinspection PyPep8Naming,PyUnresolvedReferences
    import Queue as queue

logger = getLogger(__name__)

try:
    import can
except ImportError:
    PythonCAN = None
else:
    class PythonCAN(AbstractDriver):
        TX_QUEUE_SIZE = 1000

        def __init__(self, channel, **_extras):
            super(PythonCAN, self).__init__()

            try:
                if channel is None:
                    self._bus = can.interface.Bus() # get bus from environment's config file
                else:
                    if not 'bustype' in _extras:
                        _extras['bustype'] = 'socketcan'
                    self._bus = can.interface.Bus(channel=channel, bustype=_extras['bustype'], bitrate=_extras['bitrate'])
            except Exception as ex:
                logger.exception("Could not instantiate a python-can driver")
                raise

            self._writer_thread_should_stop = False
            self._write_queue = queue.Queue(self.TX_QUEUE_SIZE)
            self._write_feedback_queue = queue.Queue()

            self._writer_thread = threading.Thread(
                target=self._writer_thread_loop, name="pythoncan_writer"
            )
            self._writer_thread.daemon = True
            self._writer_thread.start()

            ppm = lambda x: x / 1e6
            milliseconds = lambda x: x * 1e-3

            # We're using this thing to estimate the difference between monotonic and real clocks
            # See http://stackoverflow.com/questions/35426864 (at the time of writing the question was unanswered)
            self._mono_to_real_estimator = TimestampEstimator(
                max_rate_error=ppm(100),
                fixed_delay=milliseconds(0.001),
                max_phase_error_to_resync=milliseconds(50),
            )

        def _convert_real_to_monotonic(self, value):
            mono = time.monotonic()  # est_real is the best guess about real timestamp here
            real = time.time()
            est_real = self._mono_to_real_estimator.update(mono, real)
            mono_to_real_offset = est_real - mono
            return value - mono_to_real_offset

        def _writer_thread_loop(self):
            while not self._writer_thread_should_stop:
                try:
                    frame = self._write_queue.get(timeout=0.1)
                except queue.Empty:
                    continue

                try:
                    while not self._writer_thread_should_stop:
                        try:
                            if can.__version__ >= '4.0.0':
                                msg = can.Message(
                                    arbitration_id=frame.id,
                                    is_extended_id=frame.extended,
                                    dlc=len(frame.data),
                                    data=list(frame.data),
                                )
                            else:
                                msg = can.Message(
                                    arbitration_id=frame.id,
                                    extended_id=frame.extended,
                                    dlc=len(frame.data),
                                    data=list(frame.data),
                                )
                            self._bus.send(msg)
                            self._bus.flush_tx_buffer()

                            frame.ts_monotonic = time.monotonic()
                            frame.ts_real = time.time()
                            self._write_feedback_queue.put(frame)
                        except OSError as ex:
                            if ex.errno == errno.ENOBUFS:
                                time.sleep(0.0002)
                            else:
                                raise
                        else:
                            break
                except Exception as ex:
                    self._write_feedback_queue.put(ex)

        def _check_write_feedback(self):
            try:
                item = self._write_feedback_queue.get_nowait()
            except queue.Empty:
                return

            if isinstance(item, Exception):
                raise item

            if isinstance(item, CANFrame):
                self._tx_hook(item)
            else:
                raise DriverError("Unexpected item in write feedback queue: %r" % item)

        def close(self):
            self._writer_thread_should_stop = True
            self._writer_thread.join()
            self._bus.shutdown()

        def receive(self, timeout=None):
            self._check_write_feedback()

            timeout = -1 if timeout is None else (timeout * 1000)

            try:
                msg = self._bus.recv(timeout=timeout)
                if msg is not None:
                    ts_mono = time.monotonic()
                    ts_real = time.time()

                    if ts_real and not ts_mono:
                        ts_mono = self._convert_real_to_monotonic(ts_real)

                    # timestamps are ugly still
                    frame = CANFrame(
                        msg.arbitration_id,
                        msg.data,
                        True,
                        ts_monotonic=ts_mono,
                        ts_real=ts_real,
                    )
                    self._rx_hook(frame)
                    return frame
            except Exception as ex:
                logger.error("Receive exception", exc_info=True)

        def send_frame(self, frame):
            if frame.canfd:
                raise DriverError('CANFD not supported by PythonCAN')
            self._check_write_feedback()
            try:
                self._write_queue.put_nowait(frame)
            except queue.Full:
                raise TxQueueFullError()
