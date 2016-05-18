# Copyright (C) 2015-2016  Intel Corporation. All rights reserved.
#
# This file is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the
# Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This file is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.
'''calibration simulation command handling'''

from __future__ import division, print_function
import math
from pymavlink import quaternion
import random
import time

from MAVProxy.modules.lib import mp_module

class CalController(object):
    def __init__(self, mpstate):
        self.mpstate = mpstate
        self.active = False
        self.reset()

    def reset(self):
        self.desired_quaternion = None
        self.general_state = 'idle'
        self.attitude_callback = None
        self.desired_quaternion_close_count = 0

    def start(self):
        self.active = True

    def stop(self):
        self.reset()
        self.mpstate.functions.process_stdin('servo set 5 1000')
        self.active = False

    def normalize_attitude_angle(self, angle):
        if angle < 0:
            angle = 2 * math.pi + angle % (-2 * math.pi)
        angle %= 2 * math.pi
        if angle > math.pi:
            return angle % -math.pi
        return angle

    def set_attitute(self, roll, pitch, yaw, callback=None):
        roll = self.normalize_attitude_angle(roll)
        pitch = self.normalize_attitude_angle(pitch)
        yaw = self.normalize_attitude_angle(yaw)

        self.desired_quaternion = quaternion.Quaternion((roll, pitch, yaw))
        self.desired_quaternion.normalize()

        scale = 500.0 / math.pi

        roll_pwm = 1500 + int(roll * scale)
        pitch_pwm = 1500 + int(pitch * scale)
        yaw_pwm = 1500 + int(yaw * scale)

        self.mpstate.functions.process_stdin('servo set 5 1150')
        self.mpstate.functions.process_stdin('servo set 6 %d' % roll_pwm)
        self.mpstate.functions.process_stdin('servo set 7 %d' % pitch_pwm)
        self.mpstate.functions.process_stdin('servo set 8 %d' % yaw_pwm)

        self.general_state = 'attitude'
        self.desired_quaternion_close_count = 0

        if callback:
            self.attitude_callback = callback

    def angvel(self, x, y, z, theta):
        m = max(abs(x), abs(y), abs(z))
        if not m:
            x_pwm = y_pwm = z_pwm = 1500
        else:
            x_pwm = 1500 + round((x / m) * 500)
            y_pwm = 1500 + round((y / m) * 500)
            z_pwm = 1500 + round((z / m) * 500)

        max_theta = 2 * math.pi
        if theta < 0:
            theta = 0
        elif theta > max_theta:
            theta = max_theta
        theta_pwm = 1200 + round((theta / max_theta) * 800)

        self.mpstate.functions.process_stdin('servo set 5 %d' % theta_pwm)
        self.mpstate.functions.process_stdin('servo set 6 %d' % x_pwm)
        self.mpstate.functions.process_stdin('servo set 7 %d' % y_pwm)
        self.mpstate.functions.process_stdin('servo set 8 %d' % z_pwm)

        self.general_state = 'angvel'

    def handle_simstate(self, m):
        if self.general_state == 'attitude':
            q = quaternion.Quaternion((m.roll, m.pitch, m.yaw))
            q.normalize()
            d1 = abs(self.desired_quaternion.q - q.q)
            d2 = abs(self.desired_quaternion.q + q.q)
            if (d1 <= 1e-2).all() or (d2 <= 1e-2).all():
                self.desired_quaternion_close_count += 1
            else:
                self.desired_quaternion_close_count = 0

            if self.desired_quaternion_close_count == 5:
                self.general_state = 'idle'
                if callable(self.attitude_callback):
                    self.attitude_callback()
                    self.attitude_callback = None

    def mavlink_packet(self, m):
        if not self.active:
            return
        if m.get_type() == 'SIMSTATE':
            self.handle_simstate(m)


class AccelcalController(CalController):
    state_data = {
        'level': dict(
            name='Level',
            attitude=(0, 0, 0),
        ),
        'LEFT': dict(
            name='Left side',
            attitude=(-math.pi / 2, 0, 0),
        ),
        'RIGHT': dict(
            name='Right side',
            attitude=(math.pi / 2, 0, 0),
        ),
        'DOWN': dict(
            name='Nose down',
            attitude=(0, -math.pi / 2, 0),
        ),
        'UP': dict(
            name='Nose up',
            attitude=(0, math.pi / 2, 0),
        ),
        'BACK': dict(
            name='Back',
            attitude=(math.pi, 0, 0),
        ),
    }

    def __init__(self, mpstate):
        super(AccelcalController, self).__init__(mpstate)
        self.state = None

    def reset(self):
        super(AccelcalController, self).reset()

    def start(self):
        super(AccelcalController, self).start()
        if self.state:
            self.set_side_state(self.state)

    def side_from_msg(self, m):
        text = str(m.text)
        if text.startswith('Place '):
            for side in self.state_data:
                if side in text:
                    return side
        return None

    def report_from_msg(self, m):
        '''Return true if successful, false if failed, None if unknown'''
        text = str(m.text)
        if 'Calibration successful' in text:
            return True
        elif 'Calibration FAILED' in text:
            return False
        return None

    def set_side_state(self, side):
        self.state = side

        if not self.active:
            return

        data = self.state_data[side]

        def callback():
            self.mpstate.console.set_status(
                name='sitl_accelcal',
                text='sitl_accelcal: %s ready - Waiting for user input' % data['name'],
                row=4,
                fg='blue',
            )
            self.mpstate.console.writeln('sitl_accelcal: attitude detected, please press any key..')

        self.mpstate.console.writeln('sitl_accelcal: sending attitude, please wait..')

        roll, pitch, yaw = data['attitude']
        self.set_attitute(roll, pitch, yaw, callback=callback)

        self.mpstate.console.set_status(
            name='sitl_accelcal',
            text='sitl_accelcal: %s - Waiting for attitude' % data['name'],
            row=4,
            fg='orange',
        )

    def mavlink_packet(self, m):
        super(AccelcalController, self).mavlink_packet(m)

        if m.get_type() != 'STATUSTEXT':
            return

        side = self.side_from_msg(m)
        if side:
            self.set_side_state(side)
        else:
            success = self.report_from_msg(m)
            if success is None:
                return

            self.state = None
            if success:
                self.mpstate.console.set_status(
                    name='sitl_accelcal',
                    text='sitl_accelcal: Calibration successful',
                    row=4,
                    fg='blue',
                )
            else:
                self.mpstate.console.set_status(
                    name='sitl_accelcal',
                    text='sitl_accelcal: Calibration failed',
                    row=4,
                    fg='red',
                )


class MagcalController(CalController):
    yaw_increment = math.radians(45)
    yaw_noise_range = math.radians(5)

    rotation_angspeed = math.pi / 4
    '''rotation angular speed in rad/s'''
    rotation_angspeed_noise = math.radians(2)
    rotation_axes = (
        (1, 0, 0),
        (0, 1, 0),
        (1, 1, 0),
    )

    full_turn_time = 2 * math.pi / rotation_angspeed

    max_full_turns = 3
    '''maximum number of full turns to be performed for each initial attitude'''

    def reset(self):
        super(MagcalController, self).reset()
        self.yaw = 0
        self.rotation_start_time = 0
        self.last_progress = {}
        self.rotation_axis_idx = 0

    def start(self):
        super(MagcalController, self).start()
        self.set_attitute(0, 0, 0, callback=self.next_rot_att_callback)

    def next_rot_att_callback(self):
        x, y, z = self.rotation_axes[self.rotation_axis_idx]
        angspeed = self.rotation_angspeed
        angspeed += random.uniform(-1, 1) * self.rotation_angspeed_noise
        self.angvel(x, y, z, angspeed)
        self.rotation_start_time = time.time()

    def next_rotation(self):
        self.rotation_axis_idx += 1
        self.rotation_axis_idx %= len(self.rotation_axes)

        if self.rotation_axis_idx == 0:
            yaw_inc = self.yaw_increment
            yaw_inc += random.uniform(-1, 1) * self.yaw_noise_range
            self.yaw = (self.yaw + yaw_inc) % (2 * math.pi)

        self.rotation_start_time = 0
        self.last_progress = {}
        self.set_attitute(0, 0, self.yaw, callback=self.next_rot_att_callback)

    def mavlink_packet(self, m):
        super(MagcalController, self).mavlink_packet(m)

        if not self.active:
            return

        if m.get_type() == 'MAG_CAL_REPORT':
            # NOTE: This may be not the ideal way to handle it
            if m.compass_id in self.last_progress:
                del self.last_progress[m.compass_id]
            if not self.last_progress:
                self.stop()
            return

        if m.get_type() != 'MAG_CAL_PROGRESS':
            return

        if not self.rotation_start_time:
            return

        t = time.time()
        m.time = t

        if m.compass_id not in self.last_progress:
            self.last_progress[m.compass_id] = m
            m.stuck = False
            return

        last = self.last_progress[m.compass_id]

        dt = t - self.rotation_start_time
        if dt > self.max_full_turns * self.full_turn_time:
            self.next_rotation()
            return

        if m.completion_pct == last.completion_pct:
            if m.time - last.time > self.full_turn_time / 2:
                last.stuck = True
        else:
            self.last_progress[m.compass_id] = m
            m.stuck = False

        for p in self.last_progress.values():
            if not p.stuck:
                break
        else:
            self.next_rotation()


class SitlCalibrationModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(SitlCalibrationModule, self).__init__(mpstate, "sitl_calibration")
        self.add_command(
            'sitl_attitude',
            self.cmd_sitl_attitude,
            'set the vehicle at the inclination given by ROLL, PITCH and YAW' +
            ' in degrees',
        )
        self.add_command(
            'sitl_angvel',
            self.cmd_angvel,
            'apply angular velocity on the vehicle with a rotation axis and a '+
            'magnitude in degrees/s',
        )
        self.add_command(
            'sitl_accelcal',
            self.cmd_sitl_accelcal,
            'actuate on the simulator vehicle for accelerometer calibration',
        )
        self.add_command(
            'sitl_magcal',
            self.cmd_sitl_magcal,
            'actuate on the simulator vehicle for magnetometer calibration',
        )
        self.add_command(
            'sitl_stop',
            self.cmd_sitl_stop,
            'stop the current calibration control',
        )

        self.controllers = dict(
            generic_controller=CalController(mpstate),
            accelcal_controller=AccelcalController(mpstate),
            magcal_controller=MagcalController(mpstate),
        )

        self.current_controller = None

    def set_controller(self, controller):
        if self.current_controller:
            self.current_controller.stop()

        controller = self.controllers.get(controller, None)
        if controller:
            controller.start()
        self.current_controller = controller

    def cmd_sitl_attitude(self, args):
        if len(args) != 3:
            print('Usage: sitl_attitude <ROLL> <PITCH> <YAW>')
            return

        try:
            roll, pitch, yaw = args
            roll = math.radians(float(roll))
            pitch = math.radians(float(pitch))
            yaw = math.radians(float(yaw))
        except ValueError:
            print('Invalid arguments')

        self.set_controller('generic_controller')
        self.current_controller.set_attitute(roll, pitch, yaw)

    def cmd_angvel(self, args):
        if len(args) != 4:
            print('Usage: sitl_angvel <AXIS_X> <AXIS_Y> <AXIS_Z> <THETA>')
            return

        try:
            x, y, z, theta = args
            x = float(x)
            y = float(y)
            z = float(z)
            theta = math.radians(float(theta))
        except ValueError:
            print('Invalid arguments')

        self.set_controller('generic_controller')
        self.current_controller.angvel(x, y, z, theta)

    def cmd_sitl_stop(self, args):
        self.set_controller('generic_controller')

    def cmd_sitl_accelcal(self, args):
        self.set_controller('accelcal_controller')

    def cmd_sitl_magcal(self, args):
        self.set_controller('magcal_controller')

    def mavlink_packet(self, m):
        for c in self.controllers.values():
            c.mavlink_packet(m)

def init(mpstate):
    '''initialise module'''
    return SitlCalibrationModule(mpstate)
