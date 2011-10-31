# fly ArduCopter in SIL

import util, pexpect, sys, time, math, shutil

sys.path.insert(0, 'pymavlink')
import mavutil

HOME_LOCATION='-35.362938,149.165085,650,270'

def arm_motors(mavproxy):
    '''arm motors'''
    mavproxy.send('switch 6\n') # stabilize mode
    mavproxy.expect('STABILIZE>')
    mavproxy.send('rc 3 1000\n')
    mavproxy.send('rc 4 2000\n')
    mavproxy.expect('APM: ARMING MOTORS')
    mavproxy.send('rc 4 1500\n')
    print("MOTORS ARMED OK")

def disarm_motors(mavproxy):
    '''disarm motors'''
    mavproxy.send('rc 3 1000\n')
    mavproxy.send('rc 4 1000\n')
    mavproxy.expect('APM: DISARMING MOTORS')
    mavproxy.send('rc 4 1500\n')
    print("MOTORS DISARMED OK")
    

def takeoff(mavproxy, mav):
    '''takeoff get to 30m altitude'''
    mavproxy.send('switch 6\n') # stabilize mode
    mavproxy.expect('STABILIZE>')
    mavproxy.send('rc 3 1500\n')
    mavproxy.send('status\n')
    m = mav.recv_match(type='VFR_HUD', condition='VFR_HUD.alt>=30', blocking=True)
    print("Altitude %u" % m.alt)
    print("TAKEOFF COMPLETE")


def loiter(mavproxy, mav, maxaltchange=10, holdtime=10, timeout=60):
    '''hold loiter position'''
    mavproxy.send('switch 5\n') # loiter mode
    mavproxy.expect('LOITER>')
    mavproxy.send('status\n')
    mavproxy.expect('>')
    m = mav.recv_match(type='VFR_HUD', blocking=True)
    start_altitude = m.alt
    tstart = time.time()
    tholdstart = time.time()
    print("Holding loiter at %u meters for %u seconds" % (start_altitude, holdtime))
    while time.time() < tstart + timeout:
        m = mav.recv_match(type='VFR_HUD', blocking=True)
        print("Altitude %u" % m.alt)
        if math.fabs(m.alt - start_altitude) > maxaltchange:
            tholdstart = time.time()
        if time.time() - tholdstart > holdtime:
            print("Loiter OK for %u seconds" % holdtime)
            return True
    print("Loiter FAILED")
    return False


def land(mavproxy, mav, timeout=60):
    '''land the quad'''
    mavproxy.send('switch 6\n')
    mavproxy.expect('STABILIZE>')
    mavproxy.send('status\n')
    mavproxy.expect('>')
    mavproxy.send('rc 3 1300\n')
    tstart = time.time()
    while time.time() < tstart + timeout:
        m = mav.recv_match(type='VFR_HUD', blocking=True)
        print("Altitude %u" % m.alt)
        if m.alt <= 0:
            print("LANDED OK")
            return True
    print("LANDING FAILED")
    return False
    
        

def setup_rc(mavproxy):
    '''setup RC override control'''
    for chan in range(1,9):
        mavproxy.send('rc %u 1500\n' % chan)
    # zero throttle
    mavproxy.send('rc 3 1000\n')


def fly_ArduCopter():
    '''fly ArduCopter in SIL'''
    util.rmfile('eeprom.bin')
    sil = util.start_SIL('ArduCopter')
    mavproxy = util.start_MAVProxy_SIL('ArduCopter')
    mavproxy.expect('Please Run Setup')
    # we need to restart it after eeprom erase
    mavproxy.close()
    sil.close()
    sil = util.start_SIL('ArduCopter')
    mavproxy = util.start_MAVProxy_SIL('ArduCopter', options='--fgout=127.0.0.1:5502 --fgin=127.0.0.1:5501 --out=127.0.0.1:14550 --quadcopter')
    mavproxy.expect('Logging to (\S+)')
    logfile = mavproxy.match.group(1)
    print("LOGFILE %s" % logfile)
    mavproxy.expect("Ready to FLY")
    mavproxy.expect('Received [0-9]+ parameters')

    # setup test parameters
    mavproxy.send("param load autotest/ArduCopter.parm\n")
    mavproxy.expect('Loaded [0-9]+ parameters')

    # start hil_quad.py
    hquad = pexpect.spawn('HILTest/hil_quad.py --fgout=192.168.2.15:9123 --home=%s' % HOME_LOCATION,
                        logfile=sys.stdout, timeout=10)
    hquad.expect('Starting at')

    # get a mavlink connection going
    mav = mavutil.mavlink_connection('127.0.0.1:14550', robust_parsing=True)

    failed = False
    try:
        mav.wait_heartbeat()
        mav.recv_match(type='GPS_RAW')
        setup_rc(mavproxy)
        arm_motors(mavproxy)
        takeoff(mavproxy, mav)
        loiter(mavproxy, mav)
        land(mavproxy, mav)
        disarm_motors(mavproxy)
    except Exception, e:
        failed = True

    mavproxy.close()
    sil.close()
    hquad.close()

    shutil.copy(logfile, "buildlogs/ArduCopter-test.mavlog")
    util.run_cmd("pymavlink/examples/mavtogpx.py buildlogs/ArduCopter-test.mavlog")
    util.run_cmd("bin/gpxtokml buildlogs/ArduCopter-test.mavlog.gpx")
    
    if failed:
        print("FAILED: %s" % e)
        sys.exit(1)
        
