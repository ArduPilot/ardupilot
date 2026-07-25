#
# Copyright (C) 2022 DroneCAN Development Team  <uavcan.org>
#
# This software is distributed under the terms of the MIT License.
#
'''
 driver for CAN over MAVLink, using MAV_CMD_CAN_FORWARD and CAN_FRAME messages
'''

import os
import sys
import time
import multiprocessing
from logging import getLogger
from .common import DriverError, CANFrame, AbstractDriver
from pymavlink import mavutil

try:
    import queue
except ImportError:
    # noinspection PyPep8Naming,PyUnresolvedReferences
    import Queue as queue

if 'darwin' in sys.platform:
    RX_QUEUE_SIZE = 32767   # http://stackoverflow.com/questions/5900985/multiprocessing-queue-maxsize-limit-is-32767
else:
    RX_QUEUE_SIZE = 1000000
TX_QUEUE_SIZE = 1000

logger = getLogger(__name__)
kill_process = False
class ControlMessage(object):
    def __init__(self, command, data):
        self.command = command
        self.data = data

def io_process(url, bus, target_system, baudrate, tx_queue, rx_queue, exit_queue, parent_pid):
    os.environ['MAVLINK20'] = '1'

    target_component = 0
    last_enable = time.time()
    conn = None
    filter_list = None
    signing_key = None
    last_loss_print_t = time.time()
    exit_proc = False
    readonly = False

    def connect():
        nonlocal conn, baudrate, readonly
        conn = mavutil.mavlink_connection(url, baud=baudrate, source_system=250,
                                          source_component=mavutil.mavlink.MAV_COMP_ID_MAVCAN,
                                          dialect='ardupilotmega')
        if conn is None:
            raise DriverError('unable to connect to %s' % url)
        nonlocal signing_key
        if signing_key is not None:
            conn.setup_signing(signing_key, sign_outgoing=True)

        readonly = isinstance(conn, mavutil.mavlogfile)

    def reconnect():
        nonlocal exit_proc
        while True and not exit_proc:
            if (not exit_queue.empty() and exit_queue.get() == "QUIT"):
                exit_proc = True
                return
            try:
                time.sleep(1)
                logger.info('reconnecting to %s' % url)
                connect()
                return
            except Exception:
                continue

    def enable_can_forward():
        '''re-enable CAN forwarding. Called at 1Hz'''
        if readonly:
            return
        nonlocal last_enable, bus, target_system
        last_enable = time.time()
        conn.mav.command_long_send(
            target_system,
            target_component,
            mavutil.mavlink.MAV_CMD_CAN_FORWARD,
            0,
            bus+1,
            0,
            0,
            0,
            0,
            0,
            0)
        if filter_list is not None:
            ids = sorted(filter_list[:16])
            num_ids = len(ids)
            if len(ids) < 16:
                ids += [0]*(16-num_ids)
            try:
                conn.mav.can_filter_modify_send(
                    target_system,
                    target_component,
                    bus+1,
                    mavutil.mavlink.CAN_FILTER_REPLACE,
                    num_ids,
                    ids)
            except Exception as ex:
                print(ex)

    def handle_control_message(m):
        '''handle a ControlMessage'''
        if m.command == "BusNum":
            nonlocal bus
            bus = int(m.data)
        elif m.command == "FilterList":
            nonlocal filter_list
            filter_list = m.data
        elif m.command == "SigningKey":
            nonlocal signing_key
            signing_key = m.data
            conn.setup_signing(signing_key, sign_outgoing=True)

    connect()
    enable_can_forward()

    while True:
        if (not exit_queue.empty() and exit_queue.get() == "QUIT") or exit_proc:
            conn.close()
            return
        if os.getppid() != parent_pid:
            # ensure we die when parent dies
            conn.close()
            return
        while not tx_queue.empty():
            if (not exit_queue.empty() and exit_queue.get() == "QUIT") or exit_proc:
                conn.close()
                return
            frame = tx_queue.get()
            if readonly:
                continue
            if isinstance(frame, ControlMessage):
                handle_control_message(frame)
                continue
            message_id = frame.id
            if frame.extended:
                message_id |= 1<<31
            message = frame.data
            mlen = len(message)
            if mlen < frame.MAX_DATA_LENGTH:
                message += bytearray([0]*(frame.MAX_DATA_LENGTH-mlen))
            try:
                if frame.canfd:
                    conn.mav.canfd_frame_send(
                        target_system,
                        target_component,
                        bus,
                        mlen,
                        message_id,
                        message)
                else:
                    conn.mav.can_frame_send(
                        target_system,
                        target_component,
                        bus,
                        mlen,
                        message_id,
                        message)
            except Exception as ex:
                print(ex)
            if time.time() - last_enable > 1:
                enable_can_forward()

        try:
            m = conn.recv_match(type=['CAN_FRAME','CANFD_FRAME'],blocking=True,timeout=0.005)
        except Exception as ex:
            reconnect()
            continue
        now = time.time()
        if m is None:
            if now - last_enable > 1:
                enable_can_forward()
            if now - last_loss_print_t > 5:
                last_loss_print_t = now
                print("MAVLink packet loss %.2f%%" % conn.packet_loss())
            continue
        if target_system == 0:
            target_system = m.get_srcSystem()
        is_extended = (m.id & (1<<31)) != 0
        is_canfd = m.get_type() == 'CANFD_FRAME'
        canid = m.id & 0x1FFFFFFF
        frame = CANFrame(canid, m.data[:m.len], is_extended, canfd=is_canfd)
        rx_queue.put_nowait(frame)


# MAVLink CAN driver
#
class MAVCAN(AbstractDriver):
    """
    Driver for MAVLink CAN bus adapters, using CAN_FRAME MAVLink packets
    """

    def __init__(self, url, **kwargs):
        super(MAVCAN, self).__init__()
        self.bus = kwargs.get('bus_number', 1) - 1
        self.target_system = kwargs.get('mavlink_target_system', 0)
        self.filter_list = None
        baudrate = kwargs.get('baudrate', 115200)

        self.rx_queue = multiprocessing.Queue(maxsize=RX_QUEUE_SIZE)
        self.tx_queue = multiprocessing.Queue(maxsize=TX_QUEUE_SIZE)
        self.exit_queue = multiprocessing.Queue(maxsize=1)

        self.proc = multiprocessing.Process(target=io_process, name='mavcan_io_process',
                                            args=(url, self.bus, self.target_system, baudrate,
                                            self.tx_queue, self.rx_queue, self.exit_queue, os.getpid()))
        self.proc.daemon = True
        self.proc.start()

        # allow signing pass phrase in environment
        pass_phrase = os.environ.get("DRONECAN_SIGNING_KEY",None)
        if pass_phrase:
            self.set_signing_passphrase(pass_phrase)

    def close(self):
        if self.proc is not None:
            self.exit_queue.put_nowait("QUIT")
            self.proc.join()

    def __del__(self):
        self.close()

    def receive(self, timeout=None):
        tstart = time.time()
        while True:
            try:
                frame = self.rx_queue.get(block=0)
            except queue.Empty:
                frame = None
            if frame is not None:
                self._rx_hook(frame)
                return frame
            if timeout is not None:
                timeout = max(timeout, 0.001)
                if time.time() >= tstart + timeout:
                    return

    def send_frame(self, frame):
        self._tx_hook(frame)
        self.tx_queue.put_nowait(frame)

    def is_mavlink_port(device_name, baudrate):
        '''check if a device is sending mavlink'''
        os.environ['MAVLINK20'] = '1'
        conn = mavutil.mavlink_connection(device_name, baud=baudrate, source_system=250, source_component=mavutil.mavlink.MAV_COMP_ID_MAVCAN)
        if not conn:
            return False
        m = conn.recv_match(blocking=True, type=['HEARTBEAT','ATTITUDE', 'SYS_STATUS'], timeout=1.1)
        conn.close()
        return m is not None

    def set_filter_list(self, ids):
        '''set list of message IDs to accept, sent to the remote capture node with mavcan'''
        self.filter_list = ids
        self.tx_queue.put_nowait(ControlMessage('FilterList', self.filter_list))

    def get_filter_list(self, ids):
        '''set list of message IDs to accept, sent to the remote capture node with mavcan'''
        return self.filter_list

    def set_bus(self, busnum):
        '''set the remote bus number to attach to'''
        if busnum <= 0:
            raise DriverError('invalid bus %s' % busnum)
        self.bus = busnum - 1
        self.tx_queue.put_nowait(ControlMessage('BusNum', self.bus))

    def get_bus(self):
        '''get the remote bus number we are attached to'''
        return self.bus+1

    def get_filter_list(self):
        '''get the current filter list'''
        return self.filter_list

    def passphrase_to_key(self, passphrase):
        '''convert a passphrase to a 32 byte key'''
        import hashlib
        h = hashlib.new('sha256')
        if sys.version_info[0] >= 3:
            passphrase = passphrase.encode('ascii')
        h.update(passphrase)
        return h.digest()

    def set_signing_passphrase(self, passphrase):
        '''set MAVLink2 signing passphrase'''
        signing_key = self.passphrase_to_key(passphrase)
        self.tx_queue.put_nowait(ControlMessage('SigningKey', signing_key))
