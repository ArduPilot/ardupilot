#!/usr/bin/env python
"""
simple test of wind generation code
"""
from __future__ import print_function
from __future__ import absolute_import
import time
from . import util
from .rotmat import Vector3

wind = util.Wind('7,90,0.1')

t0 = time.time()
velocity = Vector3(0, 0, 0)

t = 0
deltat = 0.01

while t < 60:
    print("%.4f %f" % (t, wind.drag(velocity, deltat=deltat).length()))
    t += deltat
