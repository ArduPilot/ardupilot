#!/usr/bin/env python
'''
mavlink python utility functions

Copyright Andrew Tridgell 2011
Released under GNU GPL version 3 or later
'''

import socket, math, struct, time, os, fnmatch, array, sys, errno
from math import *
from mavextra import *

if os.getenv('MAVLINK10'):
    import mavlinkv10 as mavlink
else:
    import mavlink

def evaluate_expression(expression, vars):
    '''evaluation an expression'''
    try:
        v = eval(expression, globals(), vars)
    except NameError:
        return None
    return v

def evaluate_condition(condition, vars):
    '''evaluation a conditional (boolean) statement'''
    if condition is None:
        return True
    v = evaluate_expression(condition, vars)
    if v is None:
        return False
    return v


class mavfile(object):
    '''a generic mavlink port'''
    def __init__(self, fd, address, source_system=255):
        self.fd = fd
        self.address = address
        self.messages = { 'MAV' : self }
        if mavlink.WIRE_PROTOCOL_VERSION == "1.0":
            self.messages['HOME'] = mavlink.MAVLink_gps_raw_int_message(0,0,0,0,0,0,0,0,0,0)
            mavlink.MAVLink_waypoint_message = mavlink.MAVLink_mission_item_message
        else:
            self.messages['HOME'] = mavlink.MAVLink_gps_raw_message(0,0,0,0,0,0,0,0,0)
        self.params = {}
        self.mav = None
        self.target_system = 0
        self.target_component = 0
        self.mav = mavlink.MAVLink(self, srcSystem=source_system)
        self.mav.robust_parsing = True
        self.logfile = None
        self.logfile_raw = None
        self.param_fetch_in_progress = False
        self.param_fetch_complete = False
        self.start_time = time.time()
        self.flightmode = "UNKNOWN"
        self.timestamp = 0
        self.message_hooks = []
        self.idle_hooks = []

    def recv(self, n=None):
        '''default recv method'''
        raise RuntimeError('no recv() method supplied')

    def close(self, n=None):
        '''default close method'''
        raise RuntimeError('no close() method supplied')

    def write(self, buf):
        '''default write method'''
        raise RuntimeError('no write() method supplied')

    def pre_message(self):
        '''default pre message call'''
        return

    def post_message(self, msg):
        '''default post message call'''
        msg._timestamp = time.time()
        type = msg.get_type()
        self.messages[type] = msg
        self.timestamp = msg._timestamp
        if type == 'HEARTBEAT':
            self.target_system = msg.get_srcSystem()
            self.target_component = msg.get_srcComponent()
            if mavlink.WIRE_PROTOCOL_VERSION == '1.0':
                self.flightmode = mode_string_v10(msg)
        elif type == 'PARAM_VALUE':
            self.params[str(msg.param_id)] = msg.param_value
            if msg.param_index+1 == msg.param_count:
                self.param_fetch_in_progress = False
                self.param_fetch_complete = True
        elif type == 'SYS_STATUS' and mavlink.WIRE_PROTOCOL_VERSION == '0.9':
            self.flightmode = mode_string_v09(msg)
        elif type == 'GPS_RAW':
            if self.messages['HOME'].fix_type < 2:
                self.messages['HOME'] = msg
        for hook in self.message_hooks:
            hook(self, msg)


    def recv_msg(self):
        '''message receive routine'''
        self.pre_message()
        while True:
            n = self.mav.bytes_needed()
            s = self.recv(n)
            if len(s) == 0 and len(self.mav.buf) == 0:
                return None
            if self.logfile_raw:
                self.logfile_raw.write(str(s))
            msg = self.mav.parse_char(s)
            if msg:
                self.post_message(msg)
                return msg
                
    def recv_match(self, condition=None, type=None, blocking=False):
        '''recv the next MAVLink message that matches the given condition'''
        while True:
            m = self.recv_msg()
            if m is None:
                if blocking:
                    for hook in self.idle_hooks:
                        hook(self)
                    time.sleep(0.01)
                    continue
                return None
            if type is not None and type != m.get_type():
                continue
            if not evaluate_condition(condition, self.messages):
                continue
            return m

    def setup_logfile(self, logfile, mode='w'):
        '''start logging to the given logfile, with timestamps'''
        self.logfile = open(logfile, mode=mode)

    def setup_logfile_raw(self, logfile, mode='w'):
        '''start logging raw bytes to the given logfile, without timestamps'''
        self.logfile_raw = open(logfile, mode=mode)

    def wait_heartbeat(self, blocking=True):
        '''wait for a heartbeat so we know the target system IDs'''
        return self.recv_match(type='HEARTBEAT', blocking=blocking)

    def param_fetch_all(self):
        '''initiate fetch of all parameters'''
        if time.time() - getattr(self, 'param_fetch_start', 0) < 2.0:
            # don't fetch too often
            return
        self.param_fetch_start = time.time()
        self.param_fetch_in_progress = True
        self.mav.param_request_list_send(self.target_system, self.target_component)

    def time_since(self, mtype):
        '''return the time since the last message of type mtype was received'''
        if not mtype in self.messages:
            return time.time() - self.start_time
        return time.time() - self.messages[mtype]._timestamp

    def param_set_send(self, parm_name, parm_value, parm_type=None):
        '''wrapper for parameter set'''
        if mavlink.WIRE_PROTOCOL_VERSION == '1.0':
            if parm_type == None:
                parm_type = mavlink.MAV_VAR_FLOAT
            self.mav.param_set_send(self.target_system, self.target_component,
                                    parm_name, parm_value, parm_type)
        else:
            self.mav.param_set_send(self.target_system, self.target_component,
                                    parm_name, parm_value)

    def waypoint_request_list_send(self):
        '''wrapper for waypoint_request_list_send'''
        if mavlink.WIRE_PROTOCOL_VERSION == '1.0':
            self.mav.mission_request_list_send(self.target_system, self.target_component)
        else:
            self.mav.waypoint_request_list_send(self.target_system, self.target_component)

    def waypoint_clear_all_send(self):
        '''wrapper for waypoint_clear_all_send'''
        if mavlink.WIRE_PROTOCOL_VERSION == '1.0':
            self.mav.mission_clear_all_send(self.target_system, self.target_component)
        else:
            self.mav.waypoint_clear_all_send(self.target_system, self.target_component)

    def waypoint_request_send(self, seq):
        '''wrapper for waypoint_request_send'''
        if mavlink.WIRE_PROTOCOL_VERSION == '1.0':
            self.mav.mission_request_send(self.target_system, self.target_component, seq)
        else:
            self.mav.waypoint_request_send(self.target_system, self.target_component, seq)

    def waypoint_set_current_send(self, seq):
        '''wrapper for waypoint_set_current_send'''
        if mavlink.WIRE_PROTOCOL_VERSION == '1.0':
            self.mav.mission_set_current_send(self.target_system, self.target_component, seq)
        else:
            self.mav.waypoint_set_current_send(self.target_system, self.target_component, seq)

    def waypoint_count_send(self, seq):
        '''wrapper for waypoint_count_send'''
        if mavlink.WIRE_PROTOCOL_VERSION == '1.0':
            self.mav.mission_count_send(self.target_system, self.target_component, seq)
        else:
            self.mav.waypoint_count_send(self.target_system, self.target_component, seq)


class mavserial(mavfile):
    '''a serial mavlink port'''
    def __init__(self, device, baud=115200, autoreconnect=False, source_system=255):
        import serial
        self.baud = baud
        self.device = device
        self.autoreconnect = autoreconnect
        self.port = serial.Serial(self.device, self.baud, timeout=0,
                                  dsrdtr=False, rtscts=False, xonxoff=False)

        try:
            fd = self.port.fileno()
        except Exception:
            fd = None
        mavfile.__init__(self, fd, device, source_system=source_system)

    def close(self):
        self.port.close()

    def recv(self,n=None):
        if n is None:
            n = self.mav.bytes_needed()
        if self.fd is None:
            waiting = self.port.inWaiting()
            if waiting < n:
                n = waiting
        return self.port.read(n)

    def write(self, buf):
        try:
            return self.port.write(buf)
        except OSError:
            if self.autoreconnect:
                self.reset()
            return -1
            
    def reset(self):
        import serial
        self.port.close()
        while True:
            try:
                self.port = serial.Serial(self.device, self.baud, timeout=1,
                                          dsrdtr=False, rtscts=False, xonxoff=False)
                try:
                    self.fd = self.port.fileno()
                except Exception:
                    self.fd = None
                return
            except Exception:
                print("Failed to reopen %s" % self.device)
                time.sleep(1)
        

class mavudp(mavfile):
    '''a UDP mavlink socket'''
    def __init__(self, device, input=True, source_system=255):
        a = device.split(':')
        if len(a) != 2:
            print("UDP ports must be specified as host:port")
            sys.exit(1)
        self.port = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_server = input
        if input:
            self.port.bind((a[0], int(a[1])))
        else:
            self.destination_addr = (a[0], int(a[1]))
        self.port.setblocking(0)
        self.last_address = None
        mavfile.__init__(self, self.port.fileno(), device, source_system=source_system)

    def close(self):
        self.port.close()

    def recv(self,n=None):
        try:
            data, self.last_address = self.port.recvfrom(300)
        except socket.error as e:
            if e.errno in [ errno.EAGAIN, errno.EWOULDBLOCK ]:
                return ""
            raise
        return data

    def write(self, buf):
        try:
            if self.udp_server:
                if self.last_address:
                    self.port.sendto(buf, self.last_address)
            else:
                self.port.sendto(buf, self.destination_addr)
        except socket.error:
            pass

    def recv_msg(self):
        '''message receive routine for UDP link'''
        self.pre_message()
        s = self.recv()
        if len(s) == 0:
            return None
        msg = self.mav.parse_buffer(s)
        if msg is not None:
            for m in msg:
                self.post_message(m)
            return msg[0]
        return None


class mavtcp(mavfile):
    '''a TCP mavlink socket'''
    def __init__(self, device, source_system=255):
        a = device.split(':')
        if len(a) != 2:
            print("TCP ports must be specified as host:port")
            sys.exit(1)
        self.port = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.destination_addr = (a[0], int(a[1]))
        self.port.connect(self.destination_addr)
        self.port.setblocking(0)
        self.port.setsockopt(socket.SOL_TCP, socket.TCP_NODELAY, 1)
        mavfile.__init__(self, self.port.fileno(), device, source_system=source_system)

    def close(self):
        self.port.close()

    def recv(self,n=None):
        if n is None:
            n = self.mav.bytes_needed()
        try:
            data = self.port.recv(n)
        except socket.error as e:
            if e.errno in [ errno.EAGAIN, errno.EWOULDBLOCK ]:
                return ""
            raise
        return data

    def write(self, buf):
        try:
            self.port.send(buf)
        except socket.error:
            pass


class mavlogfile(mavfile):
    '''a MAVLink logfile reader/writer'''
    def __init__(self, filename, planner_format=None,
                 write=False, append=False,
                 robust_parsing=True, notimestamps=False, source_system=255):
        self.filename = filename
        self.writeable = write
        self.robust_parsing = robust_parsing
        self.planner_format = planner_format
        self.notimestamps = notimestamps
        self._two64 = math.pow(2.0, 63)
        mode = 'rb'
        if self.writeable:
            if append:
                mode = 'ab'
            else:
                mode = 'wb'
        self.f = open(filename, mode)
        mavfile.__init__(self, None, filename, source_system=source_system)

    def close(self):
        self.f.close()

    def recv(self,n=None):
        if n is None:
            n = self.mav.bytes_needed()
        return self.f.read(n)

    def write(self, buf):
        self.f.write(buf)

    def pre_message(self):
        '''read timestamp if needed'''
        # read the timestamp
        if self.notimestamps:
            return
        if self.planner_format:
            tbuf = self.f.read(21)
            if len(tbuf) != 21 or tbuf[0] != '-' or tbuf[20] != ':':
                raise RuntimeError('bad planner timestamp %s' % tbuf)
            hnsec = self._two64 + float(tbuf[0:20])
            t = hnsec * 1.0e-7         # convert to seconds
            t -= 719163 * 24 * 60 * 60 # convert to 1970 base
        else:
            tbuf = self.f.read(8)
            if len(tbuf) != 8:
                return
            (tusec,) = struct.unpack('>Q', tbuf)
            t = tusec * 1.0e-6
        self._timestamp = t

    def post_message(self, msg):
        '''add timestamp to message'''
        # read the timestamp
        super(mavlogfile, self).post_message(msg)
        if self.notimestamps:
            msg._timestamp = time.time()
        else:
            msg._timestamp = self._timestamp
        if self.planner_format:
            self.f.read(1) # trailing newline
        self.timestamp = msg._timestamp

class mavchildexec(mavfile):
    '''a MAVLink child processes reader/writer'''
    def __init__(self, filename, source_system=255):
        from subprocess import Popen, PIPE
        import fcntl
        
        self.filename = filename
        self.child = Popen(filename, shell=True, stdout=PIPE, stdin=PIPE)
        self.fd = self.child.stdout.fileno()

        fl = fcntl.fcntl(self.fd, fcntl.F_GETFL)
        fcntl.fcntl(self.fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)

        fl = fcntl.fcntl(self.child.stdout.fileno(), fcntl.F_GETFL)
        fcntl.fcntl(self.child.stdout.fileno(), fcntl.F_SETFL, fl | os.O_NONBLOCK)

        mavfile.__init__(self, self.fd, filename, source_system=source_system)

    def close(self):
        self.child.close()

    def recv(self,n=None):
        try:
            x = self.child.stdout.read(1)
        except Exception:
            return ''
        return x

    def write(self, buf):
        self.child.stdin.write(buf)


def mavlink_connection(device, baud=115200, source_system=255,
                       planner_format=None, write=False, append=False,
                       robust_parsing=True, notimestamps=False, input=True):
    '''make a serial or UDP mavlink connection'''
    if device.startswith('tcp:'):
        return mavtcp(device[4:], source_system=source_system)
    if device.startswith('udp:'):
        return mavudp(device[4:], input=input, source_system=source_system)
    if device.find(':') != -1 and not device.endswith('log'):
        return mavudp(device, source_system=source_system, input=input)
    if os.path.isfile(device):
        if device.endswith(".elf"):
            return mavchildexec(device, source_system=source_system)
        else:
            return mavlogfile(device, planner_format=planner_format, write=write,
                              append=append, robust_parsing=robust_parsing, notimestamps=notimestamps,
                              source_system=source_system)
    return mavserial(device, baud=baud, source_system=source_system)

class periodic_event(object):
    '''a class for fixed frequency events'''
    def __init__(self, frequency):
        self.frequency = float(frequency)
        self.last_time = time.time()
    def trigger(self):
        '''return True if we should trigger now'''
        tnow = time.time()
        if self.last_time + (1.0/self.frequency) <= tnow:
            self.last_time = tnow
            return True
        return False


try:
    from curses import ascii
    have_ascii = True
except:
    have_ascii = False

def is_printable(c):
    '''see if a character is printable'''
    global have_ascii
    if have_ascii:
        return ascii.isprint(c)
    if isinstance(c, int):
        ic = c
    else:
        ic = ord(c)
    return ic >= 32 and ic <= 126

def all_printable(buf):
    '''see if a string is all printable'''
    for c in buf:
        if not is_printable(c) and not c in ['\r', '\n', '\t']:
            return False
    return True

class SerialPort(object):
    '''auto-detected serial port'''
    def __init__(self, device, description=None, hwid=None):
        self.device = device
        self.description = description
        self.hwid = hwid

    def __str__(self):
        ret = self.device
        if self.description is not None:
            ret += " : " + self.description
        if self.hwid is not None:
            ret += " : " + self.hwid
        return ret

def auto_detect_serial_win32(preferred_list=['*']):
    '''try to auto-detect serial ports on win32'''
    try:
        import scanwin32
        list = sorted(scanwin32.comports())
    except:
        return []
    ret = []
    for order, port, desc, hwid in list:
        for preferred in preferred_list:
            if fnmatch.fnmatch(desc, preferred) or fnmatch.fnmatch(hwid, preferred):
                ret.append(SerialPort(port, description=desc, hwid=hwid))
    if len(ret) > 0:
        return ret
    # now the rest
    for order, port, desc, hwid in list:
        ret.append(SerialPort(port, description=desc, hwid=hwid))
    return ret
        

        

def auto_detect_serial_unix(preferred_list=['*']):
    '''try to auto-detect serial ports on win32'''
    import glob
    glist = glob.glob('/dev/ttyS*') + glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*') + glob.glob('/dev/serial/by-id/*')
    ret = []
    # try preferred ones first
    for d in glist:
        for preferred in preferred_list:
            if fnmatch.fnmatch(d, preferred):
                ret.append(SerialPort(d))
    if len(ret) > 0:
        return ret
    # now the rest
    for d in glist:
        ret.append(SerialPort(d))
    return ret



def auto_detect_serial(preferred_list=['*']):
    '''try to auto-detect serial port'''
    # see if 
    if os.name == 'nt':
        return auto_detect_serial_win32(preferred_list=preferred_list)
    return auto_detect_serial_unix(preferred_list=preferred_list)

def mode_string_v09(msg):
    '''mode string for 0.9 protocol'''
    mode = msg.mode
    nav_mode = msg.nav_mode

    MAV_MODE_UNINIT = 0
    MAV_MODE_MANUAL = 2
    MAV_MODE_GUIDED = 3
    MAV_MODE_AUTO = 4
    MAV_MODE_TEST1 = 5
    MAV_MODE_TEST2 = 6
    MAV_MODE_TEST3 = 7

    MAV_NAV_GROUNDED = 0
    MAV_NAV_LIFTOFF = 1
    MAV_NAV_HOLD = 2
    MAV_NAV_WAYPOINT = 3
    MAV_NAV_VECTOR = 4
    MAV_NAV_RETURNING = 5
    MAV_NAV_LANDING = 6
    MAV_NAV_LOST = 7
    MAV_NAV_LOITER = 8
    
    cmode = (mode, nav_mode)
    mapping = {
        (MAV_MODE_UNINIT, MAV_NAV_GROUNDED)  : "INITIALISING",
        (MAV_MODE_MANUAL, MAV_NAV_VECTOR)    : "MANUAL",
        (MAV_MODE_TEST3,  MAV_NAV_VECTOR)    : "CIRCLE",
        (MAV_MODE_GUIDED, MAV_NAV_VECTOR)    : "GUIDED",
        (MAV_MODE_TEST1,  MAV_NAV_VECTOR)    : "STABILIZE",
        (MAV_MODE_TEST2,  MAV_NAV_LIFTOFF)   : "FBWA",
        (MAV_MODE_AUTO,   MAV_NAV_WAYPOINT)  : "AUTO",
        (MAV_MODE_AUTO,   MAV_NAV_RETURNING) : "RTL",
        (MAV_MODE_AUTO,   MAV_NAV_LOITER)    : "LOITER",
        (MAV_MODE_AUTO,   MAV_NAV_LIFTOFF)   : "TAKEOFF",
        (MAV_MODE_AUTO,   MAV_NAV_LANDING)   : "LANDING",
        (MAV_MODE_AUTO,   MAV_NAV_HOLD)      : "LOITER",
        (MAV_MODE_GUIDED, MAV_NAV_VECTOR)    : "GUIDED",
        (100,             MAV_NAV_VECTOR)    : "STABILIZE",
        (101,             MAV_NAV_VECTOR)    : "ACRO",
        (102,             MAV_NAV_VECTOR)    : "ALT_HOLD",
        (107,             MAV_NAV_VECTOR)    : "CIRCLE",
        (109,             MAV_NAV_VECTOR)    : "LAND",
        }
    if cmode in mapping:
        return mapping[cmode]
    return "Mode(%s,%s)" % cmode

def mode_string_v10(msg):
    '''mode string for 1.0 protocol, from heartbeat'''
    if not msg.base_mode & mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED:
        return "Mode(0x%08x)" % msg.base_mode
    mapping = {
        0 : 'MANUAL',
        1 : 'CIRCLE',
        2 : 'STABILIZE',
        5 : 'FBWA',
        6 : 'FBWB',
        7 : 'FBWC',
        10 : 'AUTO',
        11 : 'RTL',
        12 : 'LOITER',
        13 : 'TAKEOFF',
        14 : 'LAND',
        15 : 'GUIDED',
        16 : 'INITIALISING'
        }
    if msg.custom_mode in mapping:
        return mapping[msg.custom_mode]
    return "Mode(%u)" % msg.custom_mode

    

class x25crc(object):
    '''x25 CRC - based on checksum.h from mavlink library'''
    def __init__(self, buf=''):
        self.crc = 0xffff
        self.accumulate(buf)

    def accumulate(self, buf):
        '''add in some more bytes'''
        bytes = array.array('B')
        if isinstance(buf, array.array):
            bytes.extend(buf)
        else:
            bytes.fromstring(buf)
        accum = self.crc
        for b in bytes:
            tmp = b ^ (accum & 0xff)
            tmp = (tmp ^ (tmp<<4)) & 0xFF
            accum = (accum>>8) ^ (tmp<<8) ^ (tmp<<3) ^ (tmp>>4)
            accum = accum & 0xFFFF
        self.crc = accum
