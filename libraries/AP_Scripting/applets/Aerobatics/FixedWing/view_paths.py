#!/usr/bin/env python3
'''
trajectory viewer for aerobatic logs
'''

from panda3d_viewer import Viewer, ViewerConfig
from pymavlink.quaternion import QuaternionBase, Quaternion
from pymavlink.rotmat import Vector3, Matrix3
from pymavlink import mavutil
import math, sys

# quaternion to rotate to fix NED to ENU conversion
q_NED_ENU = Quaternion([0, -math.sqrt(2.0)*0.5, -math.sqrt(2)*0.5, 0])

def qtuple(q):
    '''
    return a quaternion tuple. We mirror on the Z axis by changing
    the sign of two elements to cope with the different conventions
    '''
    return (q[0],q[1],q[2],q[3])

config = ViewerConfig()
config.set_window_size(320, 240)
config.enable_antialiasing(True, multisamples=4)

def view_path(viewer, path, color):
    idx = 0
    print("Plotting %u points" % len(path))
    m = Matrix3()
    m.from_euler(0, math.radians(90), 0)
    qorient = Quaternion(m)

    for i in range(1,len(path)):
        p0 = path[i-1]
        p1 = path[i]
        dx = p1.pos[0] - p0.pos[0]
        dy = p1.pos[1] - p0.pos[1]
        dz = p1.pos[2] - p0.pos[2]
        dt = p1.t - p0.t
        if dt > 0.5:
            continue
        dist = math.sqrt(dx**2+dy**2+dz**2)+0.001
        if dist <= 0:
            continue
        if math.isnan(p1.q[0]) or math.isnan(p1.q[1]) or math.isnan(p1.q[2]) or math.isnan(p1.q[3]):
            continue
        pname = 'p%u' % i
        viewer.append_box('root', pname, (dist, 0.1, 0.002), frame=(p1.pos,qtuple(p1.q)))
        viewer.set_material('root', pname, color_rgba=color)

class LogPoint(object):
    def __init__(self, x,y,z,q,t):
        # convert from NED to ENU
        global q_NED_ENU
        self.pos = (y,x,-z)
        self.q = q_NED_ENU * q
        self.t = t

def show_log(viewer,filename):
    print("Viewing %s" % filename)
    mlog = mavutil.mavlink_connection(filename)

    path_POST = []
    path_POSM = []
    path_POSB = []
    ATT = None

    scale = 0.01

    while True:
        m = mlog.recv_match(type=['POST', 'POSB', 'POSM', 'ATT'])
        if m is None:
            break
        if m.get_type() == 'POST' and ATT is not None:
            path_POST.append(LogPoint(m.px*scale, m.py*scale, m.pz*scale, Quaternion([m.q1, m.q2, m.q3, m.q4]), m.TimeUS*1.0e-6))
        if m.get_type() == 'POSB' and ATT is not None:
            path_POSB.append(LogPoint(m.px*scale, m.py*scale, m.pz*scale, Quaternion([m.q1, m.q2, m.q3, m.q4]), m.TimeUS*1.0e-6))
        if m.get_type() == 'POSM' and ATT is not None:
            path_POSM.append(LogPoint(m.px*scale, m.py*scale, m.pz*scale, Quaternion([m.q1, m.q2, m.q3, m.q4]), m.TimeUS*1.0e-6))
        if m.get_type() == 'ATT':
            ATT = m
    view_path(viewer, path_POST, (0.7,0.1,0.1,1))
    view_path(viewer, path_POSM, (0.1,0.7,0.1,1))
    view_path(viewer, path_POSB, (0.1,0.1,0.7,1))
filename = sys.argv[1]

viewer = Viewer(window_type='onscreen', window_title=filename, config=config)
viewer.append_group('root')
show_log(viewer, sys.argv[1])
viewer.reset_camera(pos=(0, -6, 1), look_at=(0, 0, 1))
viewer.join()
