#!/usr/bin/env python3

'''
Test parameter upgrade, master vs branch

./Tools/autotest/test_param_upgrade.py --vehicle=arduplane --param "GPS_TYPE=17->GPS1_TYPE=17" --param "GPS_TYPE2=37->GPS2_TYPE=37" --param "GPS_GNSS_MODE=21->GPS1_GNSS_MODE=21" --param "GPS_GNSS_MODE2=45->GPS2_GNSS_MODE=45" --param "GPS_RATE_MS=186->GPS1_RATE_MS=186" --param "GPS_RATE_MS2=123->GPS2_RATE_MS=123" --param "GPS_POS1_X=3.75->GPS1_POS_X=3.75" --param "GPS_POS2_X=6.9->GPS2_POS_X=6.9"  --param "GPS_POS1_Y=2.75->GPS1_POS_Y=2.75" --param "GPS_POS2_Y=5.9->GPS2_POS_Y=5.9"  --param "GPS_POS1_Z=12.1->GPS1_POS_Z=12.1" --param "GPS_POS2_Z=-6.9->GPS2_POS_Z=-6.9" --param "GPS_DELAY_MS=987->GPS1_DELAY_MS=987" --param "GPS_DELAY_MS2=2345->GPS2_DELAY_MS=2345" --param "GPS_COM_PORT=19->GPS1_COM_PORT=19" --param "GPS_COM_PORT2=100->GPS2_COM_PORT=100" --param "GPS_CAN_NODEID1=109->GPS1_CAN_NODEID=109" --param "GPS_CAN_NODEID2=102->GPS2_CAN_NODEID=102" --param "GPS1_CAN_OVRIDE=34->GPS1_CAN_OVRIDE=34" --param "GPS2_CAN_OVRIDE=67" --param "GPS_MB1_TYPE=1->GPS1_MB_TYPE=1" --param "GPS_MB1_OFS_X=3.14->GPS1_MB_OFS_X=3.14"  --param "GPS_MB1_OFS_Y=2.18->GPS1_MB_OFS_Y=2.18"  --param "GPS_MB1_OFS_Z=17.6->GPS1_MB_OFS_Z=17.6"  --param "GPS_MB2_TYPE=13->GPS2_MB_TYPE=13" --param "GPS_MB2_OFS_X=17.14->GPS2_MB_OFS_X=17.14"  --param "GPS_MB2_OFS_Y=12.18->GPS2_MB_OFS_Y=12.18"  --param "GPS_MB2_OFS_Z=27.6->GPS2_MB_OFS_Z=27.6"  # noqa

./Tools/autotest/test_param_upgrade.py --vehicle=arduplane --param "SERIAL1_OPTIONS=1024->MAV2_OPTIONS=2" --param "SERIAL2_OPTIONS=4096->MAV3_OPTIONS=4"
./Tools/autotest/test_param_upgrade.py --vehicle=arduplane --param "SERIAL1_OPTIONS=5120->MAV2_OPTIONS=6"

AP_FLAKE8_CLEAN
'''

import vehicle_test_suite
import os
import sys
import argparse
import subprocess
import time
import shutil
import string
import pathlib

from pysim import util


class ParamChange():
    def __init__(self, old_name, old_value, new_name, new_value):
        self.old_name = old_name
        self.old_value = old_value
        self.new_name = new_name
        self.new_value = new_value

    def __str__(self):
        return f"{self.old_name}={self.old_value}->{self.new_name}={self.new_value}"


class TestParamUpgradeTestSuite(vehicle_test_suite.TestSuite):
    def __init__(self, binary):
        super(TestParamUpgradeTestSuite, self).__init__(binary)

    def sysid_thismav(self):
        if "antennatracker" in self.binary:
            return 2
        return super(TestParamUpgradeTestSuite, self).sysid_thismav()

    def vehicleinfo_key(self):
        '''magically guess vehicleinfo_key from filepath'''
        path = self.binary.lower()
        if "blimp" in path:
            return "Blimp"
        if "copter" in path:
            return "ArduCopter"
        if "plane" in path:
            return "ArduPlane"
        if "rover" in path:
            return "Rover"
        if "sub" in path:
            return "ArduSub"
        if "tracker" in path:
            return "AntennaTracker"
        raise ValueError(f"Can't determine vehicleinfo_key from binary path {path}")

    def model(self):
        path = self.binary.lower()
        if "blimp" in path:
            return "blimp"
        if "copter" in path:
            return "X"
        if "plane" in path:
            return "quadplane"
        if "rover" in path:
            return "rover"
        if "sub" in path:
            return "vectored"
        if "tracker" in path:
            return "tracker"
        raise ValueError(f"Can't determine model from binary path {path}")


class TestParamUpgradeTestSuiteSetParameters(TestParamUpgradeTestSuite):
    def __init__(self, binary, param_changes):
        super(TestParamUpgradeTestSuiteSetParameters, self).__init__(binary)
        self.param_changes = param_changes

    def run(self):
        self.start_SITL(
            binary=self.binary,
            model=self.model(),
            wipe=True,
            sitl_home="1,1,1,1",
        )
        self.get_mavlink_connection_going()

        sets = {}
        for param_change in param_changes:
            sets[param_change.old_name] = param_change.old_value
        self.set_parameters(sets)

        # stopping SITL too soon can leave eeprom.bin corrupt!
        self.delay_sim_time(2)  # FIXTHAT, should not be needed!

        self.stop_SITL()


class TestParamUpgradeTestSuiteCheckParameters(TestParamUpgradeTestSuite):
    def __init__(self, binary, param_changes, epsilon=0.0001):
        super(TestParamUpgradeTestSuiteCheckParameters, self).__init__(binary)
        self.param_changes = param_changes
        self.epsilon = epsilon

    def run(self):
        self.start_SITL(
            binary=self.binary,
            model=self.model(),
            sitl_home="1,1,1,1",
            wipe=False,
        )
        self.get_mavlink_connection_going()

        params_to_check = [x.new_name for x in self.param_changes]
        fetched_params = self.get_parameters(params_to_check)

        for p in self.param_changes:
            if abs(fetched_params[p.new_name] - p.new_value) > self.epsilon:
                raise ValueError(f"{p.old_name}={p.old_value} did not convert into {p.new_name}={p.new_value}, got {p.new_name}={fetched_params[p.new_name]}")  # noqa

            print(f"OK: {p.old_name}={p.old_value} converted  to {p.new_name}={p.new_value}")

        self.stop_SITL()


class TestParamUpgradeForVehicle():
    def __init__(self,
                 vehicle,
                 param_changes,
                 branch=None,
                 master_branch="master",
                 run_eedump_before=False,
                 run_eedump_after=False,
                 ):
        self.vehicle = vehicle
        self.master_branch = master_branch
        self.branch = branch
        self.param_changes = param_changes
        self.run_eedump_before = run_eedump_before
        self.run_eedump_after = run_eedump_after

    def run_program(self, prefix, cmd_list, show_output=True, env=None, show_output_on_error=True, show_command=None, cwd="."):
        if show_command is None:
            show_command = True
        if show_command:
            cmd = " ".join(cmd_list)
            if cwd is None:
                cwd = "."
            self.progress(f"Running ({cmd}) in ({cwd})")
        p = subprocess.Popen(
            cmd_list,
            stdin=None,
            stdout=subprocess.PIPE,
            close_fds=True,
            stderr=subprocess.STDOUT,
            cwd=cwd,
            env=env)
        output = ""
        while True:
            x = p.stdout.readline()
            if len(x) == 0:
                returncode = os.waitpid(p.pid, 0)
                if returncode:
                    break
                    # select not available on Windows... probably...
                time.sleep(0.1)
                continue
            x = bytearray(x)
            x = filter(lambda x : chr(x) in string.printable, x)
            x = "".join([chr(c) for c in x])
            output += x
            x = x.rstrip()
            some_output = "%s: %s" % (prefix, x)
            if show_output:
                print(some_output)
            else:
                output += some_output
        (_, status) = returncode
        if status != 0:
            if not show_output and show_output_on_error:
                # we were told not to show output, but we just
                # failed... so show output...
                print(output)
            self.progress("Process failed (%s)" %
                          str(returncode))
            try:
                path = pathlib.Path(self.tmpdir, f"process-failure-{int(time.time())}")
                path.write_text(output)
                self.progress("Wrote process failure file (%s)" % path)
            except Exception:
                self.progress("Writing process failure file failed")
            raise subprocess.CalledProcessError(
                returncode, cmd_list)
        return output

    def find_current_git_branch_or_sha1(self):
        try:
            output = self.run_git(["symbolic-ref", "--short", "HEAD"])
            output = output.strip()
            return output
        except subprocess.CalledProcessError:
            pass

        # probably in a detached-head state.  Get a sha1 instead:
        output = self.run_git(["rev-parse", "--short", "HEAD"])
        output = output.strip()
        return output

    def find_git_branch_merge_base(self, branch, master_branch):
        output = self.run_git(["merge-base", branch, master_branch])
        output = output.strip()
        return output

    def run_git(self, args, show_output=True, source_dir=None):
        '''run git with args git_args; returns git's output'''
        cmd_list = ["git"]
        cmd_list.extend(args)
        return self.run_program("TPU-GIT", cmd_list, show_output=show_output, cwd=source_dir)

    def progress(self, string):
        '''pretty-print progress'''
        print("TPU: %s" % string)

    def binary_path(self, vehicle):
        build_subdir = "sitl"
        if 'AP_Periph' in vehicle:
            build_subdir = "sitl_periph_universal"
        return f"build/{build_subdir}/{self.build_target_name(vehicle)}"

    def build_target_name(self, vehicle):
        binary_name = vehicle
        if binary_name == 'heli':
            binary_name = 'arducopter-heli'
        if binary_name == 'rover':
            binary_name = 'ardurover'
        return f"bin/{binary_name}"

    def run_eedump(self):
        self.run_program("TPU-EED", [
            "libraries/AP_Param/tools/eedump_apparam",
            "eeprom.bin"
        ])

    def run(self):
        branch = self.branch
        if branch is None:
            branch = self.find_current_git_branch_or_sha1()

        master_commit = self.master_branch

        self.use_merge_base = True
        if self.use_merge_base:
            master_commit = self.find_git_branch_merge_base(branch, self.master_branch)
            self.progress("Using merge base (%s)" % master_commit)

        self.run_git(["checkout", master_commit], show_output=False)
        self.run_git(["submodule", "update", "--recursive"], show_output=False)
        shutil.rmtree("build", ignore_errors=True)
        board = "sitl"
        if "AP_Periph" in self.vehicle:
            board = "sitl_periph_universal"
        util.build_SITL(
            self.build_target_name(self.vehicle),
            board=board,
            clean=False,
            configure=True,
        )
        suite = TestParamUpgradeTestSuiteSetParameters(self.binary_path(self.vehicle), self.param_changes)
        suite.run()

        self.run_git(["checkout", branch], show_output=False)
        self.run_git(["submodule", "update", "--recursive"], show_output=False)
        shutil.rmtree("build", ignore_errors=True)
        util.build_SITL(
            self.build_target_name(self.vehicle),
            board=board,
            clean=False,
            configure=True,
        )
        suite = TestParamUpgradeTestSuiteCheckParameters(self.binary_path(self.vehicle), self.param_changes)

        if self.run_eedump_before:
            self.run_eedump()

        # this call starts SITL which will do the upgrade:
        suite.run()

        if self.run_eedump_after:
            self.run_eedump()


class TestParamUpgrade():
    def __init__(self,
                 param_changes,
                 vehicles=None,
                 run_eedump_before=False,
                 run_eedump_after=False,
                 master_branch="master",
                 ):
        self.vehicles = vehicles
        self.param_changes = param_changes
        self.vehicles = vehicles
        self.run_eedump_before = run_eedump_before
        self.run_eedump_after = run_eedump_after
        self.master_branch = master_branch

        if self.vehicles is None:
            self.vehicles = self.all_vehicles()

    def all_vehicles(self):
        return [
            # 'AP_Periph',
            'arducopter',
            'arduplane',
            'antennatracker',
            'heli',
            'rover',
            'blimp',
            'ardusub',
        ]

    def run(self):
        for vehicle in self.vehicles:
            s = TestParamUpgradeForVehicle(
                vehicle,
                self.param_changes,
                run_eedump_before=self.run_eedump_before,
                run_eedump_after=self.run_eedump_after,
                master_branch=self.master_branch,
            )
            s.run()


if __name__ == "__main__":
    ''' main program '''
    os.environ['PYTHONUNBUFFERED'] = '1'

    if sys.platform != "darwin":
        os.putenv('TMPDIR', util.reltopdir('tmp'))

    parser = argparse.ArgumentParser("test_param_upgrade.py")
    parser.add_argument(
        "--param",
        action='append',
        default=[],
        help="PARAM=VALUE pair to test upgrade for, or PARAM=VALUE->NEWPARAM=NEWVALUE",
    )
    parser.add_argument(
        "--vehicle",
        action='append',
        default=[],
        help="vehicle to test",
    )
    parser.add_argument(
        "--run-eedump-before",
        action='store_true',
        default=False,
        help="run the (already-compiled) eedump tool on eeprom.bin before doing conversion",
    )
    parser.add_argument(
        "--run-eedump-after",
        action='store_true',
        default=False,
        help="run the (already-compiled) eedump tool on eeprom.bin after doing conversion",
    )
    parser.add_argument(
        "--master-branch",
        type=str,
        default="master",
        help="master branch to use",
    )
    args = parser.parse_args()

    param_changes = []
    for x in args.param:
        if "->" in x:
            (old, new) = x.split("->")
            (old_name, old_value) = old.split("=")
            (new_name, new_value) = new.split("=")
            param_changes.append(ParamChange(old_name, float(old_value), new_name, float(new_value)))

        else:
            (name, value) = x.split("=")
            param_changes.append(ParamChange(name, float(value), name, float(value)))

    vehicles = args.vehicle

    if 'AP_Periph' in vehicles:
        raise ValueError("AP_Periph not supported yet")

    if len(vehicles) == 0:
        vehicles = None

    x = TestParamUpgrade(
        param_changes,
        vehicles=vehicles,
        run_eedump_before=args.run_eedump_before,
        run_eedump_after=args.run_eedump_after,
        master_branch=args.master_branch,
    )
    x.run()
