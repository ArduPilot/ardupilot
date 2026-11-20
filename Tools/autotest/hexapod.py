# Copyright (C) 2026 David Buzz
# SPDX-License-Identifier: GPL-3.0-or-later
# Part of ArduPilot and released under its licence: see COPYING.txt.
'''
Walk a hexapod in SITL: a Rover with legs, whose throttle and steering a
gait script turns into strides. Only a physics backend with jointed bodies
can stand one up, so these are tests for the Godot backend.

AP_FLAKE8_CLEAN
'''

import math

from rover import AutoTestRover
from vehicle_test_suite import NotAchievedException


class AutoTestHexapod(AutoTestRover):

    def log_name(self):
        return "Hexapod"

    def vehicleinfo_key(self):
        return "Rover"

    def init(self):
        if self.frame is None:
            self.frame = 'rover-hexapod'
        if self.params is None:
            # a Rover's own defaults; the legs are the model's business
            self.params = self.model_defaults_filepath("rover")
        super(AutoTestHexapod, self).init()

    def body_height(self):
        '''how high the body really is, from the simulator and not the EKF'''
        return self.poll_message("SIM_STATE").alt

    def Walk(self):
        '''walk away from home on its own legs'''
        self.wait_ready_to_arm()
        standing = self.body_height()
        self.arm_vehicle()
        self.change_mode("MANUAL")
        # It has no wheels: the gait script turns the throttle ArduPilot
        # asks for into strides. Walking a few body lengths is the test -
        # nothing gets it there but feet pushing on the ground.
        lean = 0.0

        def watch_lean(mav, m):
            nonlocal lean
            if m.get_type() == 'ATTITUDE':
                lean = max(lean, abs(math.degrees(m.roll)), abs(math.degrees(m.pitch)))

        self.context_push()
        self.install_message_hook_context(watch_lean)
        self.set_rc(3, 1800)
        self.wait_distance(2, accuracy=1, timeout=180)
        walking = self.body_height()
        self.set_rc(3, 1500)
        self.context_pop()
        self.disarm_vehicle()
        # and it has to have got there on its feet: a robot which has
        # dropped onto its belly can still be dragged along by its legs
        self.progress("body height: %.3f m standing, %.3f m after the walk" % (standing, walking))
        if walking < standing - 0.02:
            raise NotAchievedException("walked on its belly: body %.3f m lower than it stood" %
                                       (standing - walking))
        # and level: a gait which gets there by throwing the body about
        # is not one a camera or a compass could ride on
        self.progress("most the body leaned on the way: %.1f deg" % lean)
        if lean > 5:
            raise NotAchievedException("lurched %.1f deg off level while walking" % lean)

    def tests(self):
        '''return list of all tests'''
        return [
            self.Walk,
        ]

    def default_mode(self):
        return 'MANUAL'
