#!/usr/bin/python3

'''
Run a mission in SITL

AP_FLAKE8_CLEAN
'''

import vehicle_test_suite
import os
import sys
import argparse

from pysim import util


class RunMission(vehicle_test_suite.TestSuite):
    def __init__(self, vehicle_binary, model, mission_filepath, speedup=None, sim_rate_hz=None):
        super(RunMission, self).__init__(vehicle_binary)
        self.mission_filepath = mission_filepath
        self.model = model
        self.speedup = speedup
        self.sim_rate_hz = sim_rate_hz

        if self.speedup is None:
            self.speedup = 100

    def vehicleinfo_key(self):
        '''magically guess vehicleinfo_key from filepath'''
        path = self.binary.lower()
        if "plane" in path:
            return "ArduPlane"
        if "copter" in path:
            return "ArduCopter"
        raise ValueError("Can't determine vehicleinfo_key from binary path")

    def run(self):
        self.start_SITL(
            binary=self.binary,
            model=self.model,
            sitl_home=self.sitl_home_string_from_mission_filepath(self.mission_filepath),
            speedup=self.speedup,
            sim_rate_hz=self.sim_rate_hz,
            defaults_filepath=self.model_defaults_filepath(self.model),
        )
        self.get_mavlink_connection_going()

        # hack; Plane defaults are annoying... we should do better
        # here somehow.
        if self.vehicleinfo_key() == "ArduPlane":
            self.set_parameter("RTL_AUTOLAND", 1)

        self.load_mission_from_filepath(self.mission_filepath, strict=False)
        self.change_mode('AUTO')
        self.set_streamrate(1)
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.wait_disarmed(timeout=600)
        self.stop_SITL()


if __name__ == "__main__":
    ''' main program '''
    os.environ['PYTHONUNBUFFERED'] = '1'

    if sys.platform != "darwin":
        os.putenv('TMPDIR', util.reltopdir('tmp'))

    parser = argparse.ArgumentParser("run_mission.py")
    parser.add_argument(
        'vehicle_binary',
        type=str,
        help='vehicle binary to use'
    )
    parser.add_argument(
        'model',
        type=str,
        help='vehicle model to use'
    )
    parser.add_argument(
        'mission_filepath',
        type=str,
        help='mission file path'
    )
    parser.add_argument(
        '--speedup',
        type=int,
        help='simulation speedup',
        default=None,
    )
    parser.add_argument(
        '--sim-rate-hz',
        type=int,
        help='simulation physics rate',
        default=None,
    )

    args = parser.parse_args()

    x = RunMission(
        args.vehicle_binary,
        args.model,
        args.mission_filepath,
        speedup=args.speedup,
        sim_rate_hz=args.sim_rate_hz
    )
    x.run()
