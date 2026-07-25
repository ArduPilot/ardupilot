'''
utility classes for working with DroneCAN
'''

import dronecan
import time

class DroneCANSerial(object):
    '''emulation of a serial port over DroneCAN'''
    def __init__(self, uri, target_node, target_serial_dev, lock_port=False, node=None, baudrate=230400, node_id=100, timeout=2.0):
        """Constructor.

        :param uri: DroneCAN URI
        :param target_node: CAN node number hosting serial port
        :param target_serial_dev: serial device number or -1 for automatic selection
        :param lock_port: lock the serial port while in use
        :param node: DroneCAN node, if None then one is created
        :param baudrate: initial baud rate
        :param node_id: local node id if node not supplied
        :param timeout: timeout of read operations

        """
        self.buf = bytearray()
        self.timeout = timeout
        self.target_node = target_node
        self.target_serial_dev = target_serial_dev
        self.node_id = node_id
        self.lock_port = lock_port
        self.node = node
        if node is None:
            self.node = dronecan.make_node(uri, node_id=self.node_id)
        self.handle = self.node.add_handler(dronecan.uavcan.tunnel.Targetted, self.handle_Targetted)
        self.baudrate = baudrate
        self.last_send = 0

    def spin(self):
        '''spin once to process packets'''
        try:
            while self.node.spin(0) > 0:
                pass
        except dronecan.transport.TransferError:
            # ignore corrupt frames
            pass

    def __del__(self):
        '''close port'''
        self.close()

    def close(self):
        '''close port'''
        self.handle.remove()
        self.node = None

    def handle_Targetted(self, msg):
        if msg.transfer.source_node_id != self.target_node:
            return
        if msg.message.target_node != self.node.node_id:
            return
        self.buf.extend(msg.message.buffer)

    def send_bytes(self, b):
        '''send some bytes'''
        n = len(b)
        if n > 120:
            n = 120
        message = dronecan.uavcan.tunnel.Targetted()
        message.protocol.protocol = 2
        message.target_node = self.target_node
        message.serial_id = self.target_serial_dev
        message.options = message.OPTION_LOCK_PORT if self.lock_port else 0
        message.baudrate = self.baudrate
        message.buffer = bytearray(b[:n])
        while True:
            # implement writes as blocking
            try:
                self.spin()
                self.node.broadcast(message)
            except dronecan.driver.common.TxQueueFullError:
                time.sleep(0.01)
                continue
            break
        self.last_send = time.time()
        return n

    def write(self, b):
        '''write some bytes'''
        total = 0
        while len(b) > 0:
            n = self.send_bytes(b)
            if n <= 0:
                break
            total += n
            b = b[n:]
        return total

    def read(self, n):
        '''read some bytes'''
        # ensure we have recent packets
        self.spin()
        if time.time() - self.last_send > 0.5:
            # keep connection alive
            self.send_bytes(bytearray())
        if len(self.buf) > 0:
            if n > len(self.buf):
                n = len(self.buf)
            ret = self.buf[:n]
            self.buf = self.buf[n:]
            return ret
        return bytearray()

    def flushInput(self):
        '''flush any pending input'''
        self.buf = bytearray()

    def setBaudrate(self, baudrate):
        '''set baudrate'''
        if self.baudrate == baudrate:
            return
        self.baudrate = baudrate
        self.flushInput()

    def flush(self):
        '''ensure data is written'''
        self.spin()
