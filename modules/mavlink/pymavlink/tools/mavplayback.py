#!/usr/bin/env python3

'''
play back a mavlink log as a FlightGear FG NET stream, and as a
realtime mavlink stream

Useful for visualising flights
'''
import os
import sys
import time
import tkinter

from pymavlink import fgFDM
from pymavlink import DFReader

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)

parser.add_argument("--planner", action='store_true', help="use planner file format")
parser.add_argument("--condition", default=None, help="select packets by condition")
parser.add_argument("--gpsalt", action='store_true', default=False, help="Use GPS altitude")
parser.add_argument("--mav10", action='store_true', default=False, help="Use MAVLink protocol 1.0")
parser.add_argument("--out", help="MAVLink output port (IP:port)",
                  action='append', default=['127.0.0.1:14550'])
parser.add_argument("--fgout", action='append', default=['127.0.0.1:5503'],
                  help="flightgear FDM NET output (IP:port)")
parser.add_argument("--baudrate", type=int, default=57600, help='baud rate')
parser.add_argument("log", metavar="LOG")
args = parser.parse_args()

if args.mav10:
    os.environ['MAVLINK10'] = '1'
from pymavlink import mavutil

filename = args.log


def LoadImage(filename):
    '''return an image from the images/ directory'''
    app_dir = os.path.dirname(os.path.realpath(__file__))
    path = os.path.join(app_dir, 'images', filename)
    return tkinter.PhotoImage(file=path)


class App(object):
    def __init__(self, filename):
        self.root = tkinter.Tk()

        self.filesize = os.path.getsize(filename)
        self.filepos = 0.0

        self.mlog = mavutil.mavlink_connection(filename, planner_format=args.planner,
                                               robust_parsing=True)
        if isinstance(self.mlog, DFReader.DFReader_binary):
            print("mavplayback.py only works on .tlog files, not BIN ('dataflash') files")
            sys.exit(1)

        self.mout = []
        for m in args.out:
            self.mout.append(mavutil.mavlink_connection(m, input=False, baud=args.baudrate))

        self.fgout = []
        for f in args.fgout:
            self.fgout.append(mavutil.mavudp(f, input=False))

        self.fdm = fgFDM.fgFDM()

        self.msg = self.mlog.recv_match(condition=args.condition)
        if self.msg is None:
            sys.exit(1)
        self.last_timestamp = getattr(self.msg, '_timestamp')

        self.paused = False

        self.topframe = tkinter.Frame(self.root)
        self.topframe.pack(side=tkinter.TOP)

        self.frame = tkinter.Frame(self.root)
        self.frame.pack(side=tkinter.LEFT)

        self.slider = tkinter.Scale(self.topframe, from_=0, to=1.0, resolution=0.01,
                                    orient=tkinter.HORIZONTAL, command=self.slew)
        self.slider.pack(side=tkinter.LEFT)

        self.clock = tkinter.Label(self.topframe,text="")
        self.clock.pack(side=tkinter.RIGHT)

        self.playback = tkinter.Spinbox(self.topframe, from_=0, to=20, increment=0.1, width=3)
        self.playback.pack(side=tkinter.BOTTOM)
        self.playback.delete(0, "end")
        self.playback.insert(0, 1)

        self.buttons = {}
        self.button('quit', 'gtk-quit.gif', self.frame.quit)
        self.button('pause', 'media-playback-pause.gif', self.pause)
        self.button('rewind', 'media-seek-backward.gif', self.rewind)
        self.button('forward', 'media-seek-forward.gif', self.forward)
        self.button('status', 'Status', self.status)
        self.flightmode = tkinter.Label(self.frame,text="")
        self.flightmode.pack(side=tkinter.RIGHT)

        self.next_message()
        self.root.mainloop()

    def button(self, name, filename, command):
        '''add a button'''
        try:
            img = LoadImage(filename)
            b = tkinter.Button(self.frame, image=img, command=command)
            b.image = img
        except Exception:
            b = tkinter.Button(self.frame, text=filename, command=command)
        b.pack(side=tkinter.LEFT)
        self.buttons[name] = b


    def pause(self):
        '''pause playback'''
        self.paused = not self.paused

    def rewind(self):
        '''rewind 10%'''
        pos = int(self.mlog.f.tell() - 0.1 * self.filesize)
        if pos < 0:
            pos = 0
        self.mlog.f.seek(pos)
        self.find_message()

    def forward(self):
        '''forward 10%'''
        pos = int(self.mlog.f.tell() + 0.1 * self.filesize)
        if pos > self.filesize:
            pos = self.filesize - 2048
        self.mlog.f.seek(pos)
        self.find_message()

    def status(self):
        '''show status'''
        for m in sorted(self.mlog.messages.keys()):
            print(str(self.mlog.messages[m]))



    def find_message(self):
        '''find the next valid message'''
        while True:
            self.msg = self.mlog.recv_match(condition=args.condition)
            if self.msg is not None and self.msg.get_type() != 'BAD_DATA':
                break
            if self.mlog.f.tell() > self.filesize - 10:
                self.paused = True
                break
        self.last_timestamp = getattr(self.msg, '_timestamp')

    def slew(self, value):
        '''move to a given position in the file'''
        if float(value) != self.filepos:
            pos = float(value) * self.filesize
            self.mlog.f.seek(int(pos))
            self.find_message()


    def next_message(self):
        '''called as each msg is ready'''

        msg = self.msg
        if msg is None:
            self.paused = True

        if self.paused:
            self.root.after(100, self.next_message)
            return

        try:
            speed = float(self.playback.get())
        except:
            speed = 0.0
        timestamp = getattr(msg, '_timestamp')

        now = time.strftime("%H:%M:%S", time.localtime(timestamp))
        self.clock.configure(text=now)

        if speed == 0.0:
            self.root.after(200, self.next_message)
        else:
            self.root.after(int(1000*(timestamp - self.last_timestamp) / speed), self.next_message)
        self.last_timestamp = timestamp

        while True:
            self.msg = self.mlog.recv_match(condition=args.condition)
            if self.msg is None and self.mlog.f.tell() > self.filesize - 10:
                self.paused = True
                return
            if self.msg is not None and self.msg.get_type() != "BAD_DATA":
                break

        pos = float(self.mlog.f.tell()) / self.filesize
        self.slider.set(pos)
        self.filepos = self.slider.get()

        if msg.get_type() != "BAD_DATA":
            for m in self.mout:
                m.write(msg.get_msgbuf())

        if msg.get_type() == "GPS_RAW":
            self.fdm.set('latitude', msg.lat, units='degrees')
            self.fdm.set('longitude', msg.lon, units='degrees')
            if args.gpsalt:
                self.fdm.set('altitude', msg.alt, units='meters')

        if msg.get_type() == "GPS_RAW_INT":
            self.fdm.set('latitude', msg.lat/1.0e7, units='degrees')
            self.fdm.set('longitude', msg.lon/1.0e7, units='degrees')
            if args.gpsalt:
                self.fdm.set('altitude', msg.alt/1.0e3, units='meters')

        if msg.get_type() == "VFR_HUD":
            if not args.gpsalt:
                self.fdm.set('altitude', msg.alt, units='meters')
            self.fdm.set('num_engines', 1)
            self.fdm.set('vcas', msg.airspeed, units='mps')

        if msg.get_type() == "ATTITUDE":
            self.fdm.set('phi', msg.roll, units='radians')
            self.fdm.set('theta', msg.pitch, units='radians')
            self.fdm.set('psi', msg.yaw, units='radians')
            self.fdm.set('phidot', msg.rollspeed, units='rps')
            self.fdm.set('thetadot', msg.pitchspeed, units='rps')
            self.fdm.set('psidot', msg.yawspeed, units='rps')

        if msg.get_type() == "RC_CHANNELS_SCALED":
            self.fdm.set("right_aileron", msg.chan1_scaled*0.0001)
            self.fdm.set("left_aileron", -msg.chan1_scaled*0.0001)
            self.fdm.set("rudder",        msg.chan4_scaled*0.0001)
            self.fdm.set("elevator",      msg.chan2_scaled*0.0001)
            self.fdm.set('rpm',           msg.chan3_scaled*0.01)

        if msg.get_type() == 'STATUSTEXT':
            print("AP: %s" % msg.text)

        if msg.get_type() == 'SYS_STATUS':
            self.flightmode.configure(text=self.mlog.flightmode)

        if msg.get_type() == "BAD_DATA":
            if mavutil.all_printable(msg.data):
                sys.stdout.write(msg.data)
                sys.stdout.flush()

        if self.fdm.get('latitude') != 0:
            for f in self.fgout:
                f.write(self.fdm.pack())


app=App(filename)
