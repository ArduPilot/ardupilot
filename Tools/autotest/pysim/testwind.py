#!/usr/bin/env python
# simple test of wind generation code

import util, euclid, time, random

wind = util.Wind('3,90,0.1')

t0 = time.time()
velocity = euclid.Vector3(0,0,0)

t = 0
deltat = 0.01

while t < 60:
    print("%.4f %f" % (t, wind.accel(velocity, deltat=deltat).magnitude()))
    t += deltat
