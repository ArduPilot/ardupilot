import os.path
import random
import sys
import time

import pexpect

MODULE_DIR = os.path.dirname(os.path.realpath(__file__))

# This is terrible.
sys.path.insert(0,
                os.path.join(MODULE_DIR, '..', '..', 'mavlink', 'pymavlink'))
import mavutil
import mavwp

# So is this.
sys.path.insert(0, os.path.join(MODULE_DIR, 'autotest'))
sys.path.insert(0, os.path.join(MODULE_DIR, 'autotest', 'pysim'))
import arducopter
import common
import util

testdir = os.path.dirname(os.path.realpath(__file__))


def error(msg, *args):
  sys.stdout.flush()
  sys.stderr.write((msg % args) + '\n')


def run_mission(mission_path, frame, home, viewerip=None):
  sim_cmd = util.reltopdir('Tools/autotest/pysim/sim_multicopter.py')
  sim_cmd += ' --frame=%s --rate=400 --home=%f,%f,%u,%u' % (
    frame, home.lat, home.lng, home.alt, home.heading)
  sim_cmd += ' --wind=6,45,.3'
  if viewerip:
    sim_cmd += ' --fgout=%s:5503' % viewerip

  sil = util.start_SIL('ArduCopter', wipe=True)
  mavproxy = util.start_MAVProxy_SIL(
    'ArduCopter',
    options='--sitl=127.0.0.1:5501 --out=127.0.0.1:19550 --quadcopter')
  mavproxy.expect('Received [0-9]+ parameters')

  # setup test parameters
  mavproxy.send('param set SYSID_THISMAV %u\n' % random.randint(100, 200))
  mavproxy.send("param load %s/autotest/ArduCopter.parm\n" % testdir)
  mavproxy.expect('Loaded [0-9]+ parameters')
  mavproxy.send('module load mmap\n')

  # reboot with new parameters
  util.pexpect_close(mavproxy)
  util.pexpect_close(sil)

  sil = util.start_SIL('ArduCopter', height=home.alt)
  print 'Executing command %s' % (sim_cmd,)
  sim = pexpect.spawn(sim_cmd, logfile=sys.stdout, timeout=10)
  sim.delaybeforesend = 0
  util.pexpect_autoclose(sim)
  options = ('--sitl=127.0.0.1:5501 --out=127.0.0.1:19550 --quadcopter '
             '--streamrate=5')
  if viewerip:
      options += ' --out=%s:14550' % viewerip
  mavproxy = util.start_MAVProxy_SIL('ArduCopter', options=options)
  mavproxy.expect('Logging to (\S+)')
  logfile = mavproxy.match.group(1)
  print 'Saving log %s' % (logfile,)

  # the received parameters can come before or after the ready to fly message
  mavproxy.expect(['Received [0-9]+ parameters', 'Ready to FLY'])
  mavproxy.expect(['Received [0-9]+ parameters', 'Ready to FLY'])

  mavproxy.send('module load mmap\n')
  util.expect_setup_callback(mavproxy, common.expect_callback)

  common.expect_list_clear()
  common.expect_list_extend([sim, sil, mavproxy])

  # get a mavlink connection going
  try:
    mav = mavutil.mavlink_connection('127.0.0.1:19550', robust_parsing=True)
  except Exception, msg:
    error("Failed to start mavlink connection on 127.0.0.1:19550" % msg)
    raise
  mav.message_hooks.append(common.message_hook)
  mav.idle_hooks.append(common.idle_hook)

  failed = False
  e = 'None'
  try:
    mav.wait_heartbeat()
    arducopter.setup_rc(mavproxy)

    print 'Calibrating level...'
    if not arducopter.calibrate_level(mavproxy, mav):
      error('Failed to calibrate level.')
      failed = True

    print 'Arming motors...'
    if not arducopter.arm_motors(mavproxy, mav):
      error('Failed to arm motors.')
      failed = True

    if (mission_path and
        arducopter.load_mission_from_file(mavproxy, mav, mission_path)):
        error('Failed to load mission %s' % (mission_path,))
        failed = True

    print 'Taking off.'
    if not arducopter.takeoff(mavproxy, mav, 10):
      error('Failed to takeoff.')

    if mission_path:
      print 'Beginning mission in...'
      for i in range(3, 0, -1):
        print i
        time.sleep(1)
      print 'Flying mission.'
      if not arducopter.fly_mission(mavproxy, mav, height_accuracy=0.5,
                                    target_altitude=10):
        error("fly_mission failed")
        failed = True
      else:
        print 'MISSION COMPLETE.'

  except pexpect.TIMEOUT, e:
    error('Command timed out: %s', e)
    failed = True

  mav.close()
  util.pexpect_close(mavproxy)
  util.pexpect_close(sil)
  util.pexpect_close(sim)
  return not failed


def main():
  mission_path = None
  if len(sys.argv) > 1:
    mission_path = sys.argv[1]
  frame = '+'
  # If we were given a mission, use its first waypoint as home.
  if mission_path:
    wploader = mavwp.MAVWPLoader()
    wploader.load(mission_path)
    wp = wploader.wp(0)
    home = mavutil.location(wp.x, wp.y, wp.z, 0)
  run_mission(mission_path, frame, home)


if __name__ == '__main__':
  main()
