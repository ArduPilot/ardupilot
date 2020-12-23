#!/usr/bin/env python
'''module for testing arm authorization'''
import time, math
from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib.mp_settings import MPSetting

class AuthModule(mp_module.MPModule):
  def __init__(self, mpstate):
    super(AuthModule, self).__init__(mpstate, "arm_auth", "arm_authorization_test_module")
    '''initialisation code'''
    self.add_command('auth', self.cmd_auth, "Authorization")

  def mavlink_packet(self, m):
    '''handle a mavlink packet'''
    if m.get_type() == 'ARM_AUTHORIZATION_REQUEST':
    	print("ARM Authorisation request received (target_system:%i) (auth_window:%i ms)" % (m.target_system,m.auth_window))

  def cmd_auth(self, args):
    if len(args) == 0:
      print("Usage: auth ena|dis (valid_time)")
      return
    if args[0] == "ena":
      if len(args) == 2:
        auth_time = int(args[1])
      else:
        auth_time = 0
      self.master.mav.arm_authorization_response_send(
        0, #accepted
        0,
        auth_time, #unlimited time
        self.target_system,
        self.target_component
        )
    elif args[0] == "dis":
      self.master.mav.arm_authorization_response_send(
        2, #denied
        0,
        0,
        self.target_system,
        self.target_component
        )

def init(mpstate):
  '''initialise module'''
  return AuthModule(mpstate)

