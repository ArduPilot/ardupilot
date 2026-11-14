#!/usr/bin/env python3

"""
Contains functions used to test the ArduPilot build_options.py structures

To extract feature sizes:

cat >> /tmp/extra-hwdef.dat <<EOF
undef AP_BARO_MS5611_ENABLED
define AP_BARO_MS5611_ENABLED 1
EOF

nice time ./Tools/autotest/test_build_options.py --board=CubeOrange --extra-hwdef=/tmp/extra-hwdef.dat --no-run-with-defaults --no-disable-all --no-enable-in-turn | tee /tmp/tbo-out  # noqa: E501
grep 'sabling.*saves' /tmp/tbo-out

 - note that a lot of the time explicitly disabling features will make the binary larger as the ROMFS includes the generated hwdef.h which will have the extra define in it  # noqa: E501

To use all of a machine's cores, run several builds at once:

./Tools/autotest/test_build_options.py --parallel=8

 - each worker builds into its own directory under build/test_build_options/,
   and its build output is kept in build/test_build_options/logs/
 - build/test_build_options/status.txt shows how far through the run is and
   what each worker is doing

AP_FLAKE8_CLEAN
"""  # noqa:E501

import fnmatch
import math
import multiprocessing
import optparse
import os
import pathlib
import queue
import re
import signal
import sys
import threading
import time
import traceback

from pysim import util

sys.path.insert(1, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
import board_list  # noqa
import extract_features  # noqa


class TestBuildOptionsResult(object):
    '''object to return results from a comparison'''

    def __init__(self, feature, vehicle, bytes_delta):
        self.feature = feature
        self.vehicle = vehicle
        self.bytes_delta = bytes_delta


class TestBuildOptions(object):
    def __init__(self,
                 match_glob=None,
                 do_step_disable_all=True,
                 do_step_disable_none=False,
                 do_step_disable_defaults=True,
                 do_step_disable_in_turn=True,
                 do_step_enable_in_turn=True,
                 build_targets=None,
                 board="CubeOrange",  # DevEBoxH7v2 also works
                 extra_hwdef=None,
                 emit_disable_all_defines=None,
                 resume=False,
                 parallel=None,
                 jobs=None,
                 stop_on_failure=False,
                 ):
        self.extra_hwdef = extra_hwdef
        self.sizes_nothing_disabled = None
        self.match_glob = match_glob
        self.do_step_disable_all = do_step_disable_all
        self.do_step_disable_none = do_step_disable_none
        self.do_step_run_with_defaults = do_step_disable_defaults
        self.do_step_disable_in_turn = do_step_disable_in_turn
        self.do_step_enable_in_turn = do_step_enable_in_turn
        self.build_targets = build_targets
        if self.build_targets is None:
            self.build_targets = self.all_targets()
        self._board = board
        self.emit_disable_all_defines = emit_disable_all_defines
        self.resume = resume
        self.results = {}

        self.enable_in_turn_results = {}
        self.sizes_everything_disabled = None

        # number of builds to run at once; None means run them one
        # after the other in the main process, building into build/
        self.parallel = parallel
        if jobs is None and parallel is not None:
            jobs = os.cpu_count()
        self.jobs = jobs
        # by default carry on after a build fails, reporting every
        # failure at the end:
        self.stop_on_failure = stop_on_failure
        # (name, details) of builds which have failed:
        self.failures = []
        # set in each parallel worker process:
        self.worker_number = None
        self.progress_stream = None

    def must_have_defines_for_board(self, board):
        '''return a sorted list of defines which must always be enabled.
        Sorted as the order of a set's members varies from run to run,
        and the order of the defines changes the size of the hwdef.dat
        in ROMFS'''
        must_have_defines = {
            "CubeOrange": frozenset([
                'AP_BARO_MS5611_ENABLED',
                'AP_BARO_MS5607_ENABLED',
                'AP_COMPASS_LSM303D_ENABLED',
                'AP_COMPASS_AK8963_ENABLED',
                'AP_COMPASS_AK09916_ENABLED',
                'AP_COMPASS_ICM20948_ENABLED',
            ]),
            "CubeBlack": frozenset([
                'AP_BARO_MS5611_ENABLED',
                'AP_BARO_MS5607_ENABLED',
                'AP_COMPASS_LSM303D_ENABLED',
                'AP_COMPASS_AK8963_ENABLED',
                'AP_COMPASS_AK09916_ENABLED',
                'AP_COMPASS_ICM20948_ENABLED',
            ]),
            "Pixhawk6X-GenericVehicle": frozenset([
                "AP_BARO_BMP388_ENABLED",
                "AP_BARO_ICP201XX_ENABLED",
            ]),
        }
        return sorted(must_have_defines.get(board, frozenset([])))

    def must_have_defines(self):
        return self.must_have_defines_for_board(self._board)

    def board_object(self) -> board_list.Board:
        '''return the BoardList Board object for the current board'''
        if hasattr(self, '_board_object'):
            return self._board_object
        self._board_object = board_list.BoardList().board_by_name(self._board)
        return self._board_object

    @staticmethod
    def all_targets():
        return ['copter', 'plane', 'rover', 'antennatracker', 'sub', 'blimp']

    def progress(self, message):
        stream = sys.stderr
        if self.worker_number is not None:
            # stdout and stderr go to the build log in workers
            stream = self.progress_stream
            message = "[worker-%u] %s" % (self.worker_number, message)
        # one write per line, so lines from the parallel workers and
        # the main process do not run into each other:
        stream.write("###### %s\n" % message)
        stream.flush()

    # swiped from app.py:
    def get_build_options_from_ardupilot_tree(self):
        '''return a list of build options'''
        import importlib.util
        spec = importlib.util.spec_from_file_location(
            "build_options.py",
            os.path.join(os.path.dirname(os.path.realpath(__file__)),
                         '..', 'scripts', 'build_options.py'))
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod.BUILD_OPTIONS

    def write_defines_to_file(self, defines, filepath):
        self.write_defines_to_Path(defines, pathlib.Path(filepath))

    def write_defines_to_Path(self, defines, Path):
        lines = []
        lines.extend(["undef %s\n" % (a, ) for (a, b) in defines.items()])
        lines.extend(["define %s %s\n" % (a, b) for (a, b) in defines.items()])
        content = "".join(lines)
        Path.write_text(content)

    def get_disable_defines(self, feature, options):
        '''returns a hash of (name, value) defines to turn feature off -
        recursively gets dependencies'''
        ret = {
            feature.define: 0,
        }
        added_one = True
        while added_one:
            added_one = False
            for option in options:
                if option.define in ret:
                    continue
                if option.dependency is None:
                    continue
                for dep in option.dependency.split(','):
                    f = self.get_option_by_label(dep, options)
                    if f.define not in ret:
                        continue

                    # print("%s requires %s" % (option.define, f.define), file=sys.stderr)
                    added_one = True
                    ret[option.define] = 0
                    break
        return ret

    def update_get_enable_defines_for_feature(self, ret, feature, options):
        '''recursive function to turn on required feature and what it depends
        on'''
        ret[feature.define] = 1
        if feature.dependency is None:
            return
        for depname in feature.dependency.split(','):
            dep = None
            for f in options:
                if f.label == depname:
                    dep = f
            if dep is None:
                raise ValueError("Invalid dep (%s) for feature (%s)" %
                                 (depname, feature.label))
            self.update_get_enable_defines_for_feature(ret, dep, options)

    def get_enable_defines(self, feature, options):
        '''returns a hash of (name, value) defines to turn all features *but* feature (and whatever it depends on) on'''
        ret = self.get_disable_all_defines()
        self.update_get_enable_defines_for_feature(ret, feature, options)
        for define in self.must_have_defines_for_board(self._board):
            ret[define] = 1
        return ret

    def test_disable_feature(self, feature, options):
        defines = self.get_disable_defines(feature, options)

        if len(defines.keys()) > 1:
            self.progress("Disabling %s disables (%s)" % (
                feature.define,
                ",".join(defines.keys())))

        self.test_compile_with_defines(defines)

        self.assert_feature_not_in_code(defines, feature)

    def assert_feature_not_in_code(self, defines, feature):
        # if the feature is truly disabled then extract_features.py
        # should say so:
        for target in self.build_targets:
            path = self.target_to_elf_path(target)
            extractor = extract_features.ExtractFeatures(path)
            (compiled_in_feature_defines, not_compiled_in_feature_defines) = extractor.extract()
            for define in defines:
                # the following defines are known not to work on some
                # or all vehicles:
                feature_define_whitelist = set([
                    'AP_RANGEFINDER_ENABLED',  # only at vehicle level ATM
                    'HAL_PERIPH_SUPPORT_LONG_CAN_PRINTF',  # no symbol
                    'AP_PROXIMITY_HEXSOONRADAR_ENABLED',  # this shares symbols with AP_PROXIMITY_MR72_ENABLED
                    'AP_PROXIMITY_MR72_ENABLED',    # this shares symbols with AP_PROXIMITY_HEXSOONRADAR_ENABLED
                    'AP_RANGEFINDER_NRA24_CAN_ENABLED',
                    'AP_RANGEFINDER_HEXSOONRADAR_ENABLED',
                ])
                if target.lower() == 'sub':
                    # ArduSub has its own ModeAlthold, which is unrelated
                    # to Copter's MODE_ALTHOLD_ENABLED build option.
                    feature_define_whitelist.add('MODE_ALTHOLD_ENABLED')

                if define in compiled_in_feature_defines:
                    error = f"feature gated by {define} still compiled into ({target}); extract_features.py bug?"
                    if define in feature_define_whitelist:
                        print("warn: " + error)
                    else:
                        raise ValueError(error)

    def test_enable_feature(self, feature, options):
        defines = self.get_enable_defines(feature, options)

        enabled = list(filter(lambda x : bool(defines[x]), defines.keys()))

        if len(enabled) > 1:
            self.progress("Enabling %s enables (%s)" % (
                feature.define,
                ",".join(enabled)))

        self.test_compile_with_defines(defines)

        self.assert_feature_in_code(defines, feature)

    def define_is_whitelisted_for_feature_in_code(self, target, define):
        '''returns true if we can not expect the define to be extracted from
        the binary'''
        # the following defines are known not to work on some
        # or all vehicles:
        feature_define_whitelist = set([
            'AC_POLYFENCE_CIRCLE_INT_SUPPORT_ENABLED',  # no symbol
            'AP_RANGEFINDER_ENABLED',  # only at vehicle level ATM
            'HAL_PERIPH_SUPPORT_LONG_CAN_PRINTF',  # no symbol
            'AP_DRONECAN_VOLZ_FEEDBACK_ENABLED',  # broken, no subscriber
            'AP_DRONECAN_LOG_CIRCUIT_STATUS_ENABLED',  # no symbol
            # Baro drivers either come in because you have
            # external-probing enabled or you have them specified in
            # your hwdef.  If you're not probing and its not in your
            # hwdef then the code will be elided as unreachable
            'AP_BARO_ICM20789_ENABLED',
            'AP_BARO_ICP101XX_ENABLED',
            'AP_BARO_ICP201XX_ENABLED',
            'AP_BARO_BMP085_ENABLED',
            'AP_BARO_BMP280_ENABLED',
            'AP_BARO_BMP388_ENABLED',
            'AP_BARO_BMP581_ENABLED',
            'AP_BARO_DPS280_ENABLED',
            'AP_BARO_FBM320_ENABLED',
            'AP_BARO_KELLERLD_ENABLED',
            'AP_BARO_LPS2XH_ENABLED',
            'AP_BARO_MS5607_ENABLED',
            'AP_BARO_MS5611_ENABLED',
            'AP_BARO_MS5637_ENABLED',
            'AP_BARO_MS5837_ENABLED',
            'AP_BARO_SPL06_ENABLED',
            'AP_CAMERA_SEND_FOV_STATUS_ENABLED',  # elided unless AP_CAMERA_SEND_FOV_STATUS_ENABLED
            'AP_COMPASS_LSM9DS1_ENABLED',  # must be in hwdef, not probed
            'AP_COMPASS_MAG3110_ENABLED',  # must be in hwdef, not probed
            'AP_COMPASS_MMC5XX3_ENABLED',  # must be in hwdef, not probed
            'AP_MAVLINK_AUTOPILOT_VERSION_REQUEST_ENABLED',  # completely elided
            'AP_MAVLINK_MSG_RELAY_STATUS_ENABLED',  # no symbol available
            'AP_MAVLINK_MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES_ENABLED',  # no symbol available
            'HAL_MSP_SENSORS_ENABLED',  # no symbol available
            'AP_OSD_LINK_STATS_EXTENSIONS_ENABLED',  # FIXME: need a new define/feature
            'HAL_OSD_SIDEBAR_ENABLE',  # FIXME: need a new define/feature
            'HAL_PLUSCODE_ENABLE',  # FIXME: need a new define/feature
            'AP_SERIALMANAGER_REGISTER_ENABLED',  # completely elided without a caller
            'AP_OPTICALFLOW_ONBOARD_ENABLED',  # only instantiated on Linux
            'HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL',  # entirely elided if no user
            'AP_PLANE_BLACKBOX_LOGGING',  # entirely elided if no user
            'AP_COMPASS_AK8963_ENABLED',  # probed on a board-by-board basis, not on CubeOrange for example
            'AP_COMPASS_LSM303D_ENABLED',  # probed on a board-by-board basis, not on CubeOrange for example
            'AP_BARO_THST_COMP_ENABLED',  # compiler is optimising this symbol away
            'AP_GPS_DEBUG_LOGGING_ENABLED',  # must have a backend compiled in to be present
            'AP_MOUNT_ROI_WPNEXT_OFFSET_ENABLED',  # must have a backend compiled in to be present
        ])
        if target.lower() != "copter":
            feature_define_whitelist.add('MODE_ALTHOLD_ENABLED')
            feature_define_whitelist.add('MODE_ZIGZAG_ENABLED')
            feature_define_whitelist.add('MODE_SYSTEMID_ENABLED')
            feature_define_whitelist.add('MODE_SPORT_ENABLED')
            feature_define_whitelist.add('MODE_FOLLOW_ENABLED')
            feature_define_whitelist.add('MODE_TURTLE_ENABLED')
            feature_define_whitelist.add('MODE_GUIDED_NOGPS_ENABLED')
            feature_define_whitelist.add('MODE_FLOWHOLD_ENABLED')
            feature_define_whitelist.add('MODE_FLIP_ENABLED')
            feature_define_whitelist.add('MODE_BRAKE_ENABLED')
            feature_define_whitelist.add('MODE_THROW_ENABLED')
            feature_define_whitelist.add('AP_TEMPCALIBRATION_ENABLED')
            feature_define_whitelist.add('AC_PAYLOAD_PLACE_ENABLED')
            feature_define_whitelist.add('AP_AVOIDANCE_ENABLED')
            feature_define_whitelist.add('AP_GROUNDEFFECT_ENABLED')
            feature_define_whitelist.add('AP_WINCH_ENABLED')
            feature_define_whitelist.add('AP_WINCH_DAIWA_ENABLED')
            feature_define_whitelist.add('AP_WINCH_PWM_ENABLED')
            feature_define_whitelist.add(r'AP_MOTORS_FRAME_.*_ENABLED')
            feature_define_whitelist.add('AP_MOTORS_TRI_ENABLED')
            feature_define_whitelist.add('AP_COPTER_ADVANCED_FAILSAFE_ENABLED')
            feature_define_whitelist.add('AP_INERTIALSENSOR_FAST_SAMPLE_WINDOW_ENABLED')
            feature_define_whitelist.add('AP_COPTER_AHRS_AUTO_TRIM_ENABLED')
            feature_define_whitelist.add('AP_COPTER_CUSTOMCONTROL_ENABLED')
            feature_define_whitelist.add('AP_RC_TRANSMITTER_TUNING_ENABLED')
            feature_define_whitelist.add('AP_AVOIDANCE_ALTHOLD_ENABLED')
            feature_define_whitelist.add(r'AP_MOTORS_COAX_ENABLED')
            feature_define_whitelist.add(r'AP_MOTORS_SINGLE_ENABLED')

        if target.lower() in ['antennatracker', 'blimp', 'sub', 'plane', 'copter']:
            # plane has a dependency for AP_Follow which is not
            # declared in build_options.py; we don't compile follow
            # support for Follow into Plane unless scripting is also
            # enabled.  Copter manages to elide everything is
            # MODE_FOLLOW isn't enabled.
            feature_define_whitelist.add('AP_FOLLOW_ENABLED')

        if target.lower() != "plane":
            # only on Plane:
            feature_define_whitelist.add('AP_ICENGINE_ENABLED')
            feature_define_whitelist.add('AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED')
            feature_define_whitelist.add('AP_MAVLINK_MAV_CMD_SET_HAGL_ENABLED')
            feature_define_whitelist.add('AP_ADVANCEDFAILSAFE_ENABLED')
            feature_define_whitelist.add('AP_TUNING_ENABLED')
            feature_define_whitelist.add('HAL_LANDING_DEEPSTALL_ENABLED')
            feature_define_whitelist.add('HAL_SOARING_ENABLED')
            feature_define_whitelist.add('AP_PLANE_BLACKBOX_LOGGING')
            feature_define_whitelist.add('QAUTOTUNE_ENABLED')
            feature_define_whitelist.add('AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED')
            feature_define_whitelist.add('HAL_QUADPLANE_ENABLED')
            feature_define_whitelist.add('MODE_AUTOLAND_ENABLED')
            feature_define_whitelist.add('AP_PLANE_GLIDER_PULLUP_ENABLED')
            feature_define_whitelist.add('AP_QUICKTUNE_ENABLED')
            feature_define_whitelist.add('AP_PLANE_SYSTEMID_ENABLED')
            feature_define_whitelist.add('AP_PLANE_CUSTOMCONTROL_ENABLED')
            feature_define_whitelist.add('AP_PLANE_TAILSITTER_ENABLED')
            feature_define_whitelist.add(r'AP_MOTORS_TAILSITTER_ENABLED')

        if target.lower() not in ["plane", "copter"]:
            # only the Plane and Copter scripting bindings call set_throttle_factor:
            feature_define_whitelist.add('AP_MOTORS_FRAME_SCRIPTING_MATRIX_ENABLED')
            feature_define_whitelist.add('HAL_ADSB_ENABLED')
            feature_define_whitelist.add('AP_LANDINGGEAR_ENABLED')
            # only Plane and Copter instantiate Parachute
            feature_define_whitelist.add('HAL_PARACHUTE_ENABLED')
            # only Plane and Copter have AP_Motors:
            feature_define_whitelist.add(r'AP_MOTORS_TRI_ENABLED')
            # other vehicles do not instantiate ADSB:
            feature_define_whitelist.add('AP_ADSB_AVOIDANCE_ENABLED')
            # only Plane and Copter instantiate the Motors library,
            # required for these bindings:
            feature_define_whitelist.add('AP_SCRIPTING_BINDING_MOTORS_ENABLED')

        if target.lower() not in ["plane", "rover"]:
            # only Plane and Rover support battery watt limiting
            feature_define_whitelist.add('AP_BATTERY_WATT_MAX_ENABLED')

        if target.lower() not in ["rover", "copter"]:
            # only Rover and Copter instantiate Beacon
            feature_define_whitelist.add('AP_BEACON_ENABLED')

        if target.lower() != "rover":
            # only on Rover:
            feature_define_whitelist.add('HAL_TORQEEDO_ENABLED')
            feature_define_whitelist.add('AP_ROVER_ADVANCED_FAILSAFE_ENABLED')
            feature_define_whitelist.add('AP_ROVER_AUTO_ARM_ONCE_ENABLED')
        if target.lower() != "sub":
            # only on Sub:
            feature_define_whitelist.add('AP_BARO_KELLERLD_ENABLED')
        if target.lower() not in frozenset(["rover", "sub"]):
            # only Rover and Sub get nmea airspeed
            feature_define_whitelist.add('AP_AIRSPEED_NMEA_ENABLED')
        if target.lower() not in frozenset(["copter", "rover"]):
            feature_define_whitelist.add('HAL_SPRAYER_ENABLED')
            feature_define_whitelist.add('HAL_PROXIMITY_ENABLED')
            feature_define_whitelist.add('AP_PROXIMITY_.*_ENABLED')
            feature_define_whitelist.add('AP_OAPATHPLANNER_ENABLED')

        if target.lower() in ["blimp", "antennatracker"]:
            # no airspeed on blimp/tracker
            feature_define_whitelist.add(r'AP_AIRSPEED_.*_ENABLED')
            feature_define_whitelist.add(r'HAL_MOUNT_ENABLED')
            feature_define_whitelist.add(r'AP_MOUNT_.*_ENABLED')
            feature_define_whitelist.add(r'HAL_MOUNT_.*_ENABLED')
            feature_define_whitelist.add(r'HAL_SOLO_GIMBAL_ENABLED')
            feature_define_whitelist.add(r'AP_OPTICALFLOW_ENABLED')
            feature_define_whitelist.add(r'AP_OPTICALFLOW_.*_ENABLED')
            feature_define_whitelist.add(r'HAL_MSP_OPTICALFLOW_ENABLED')
            # missing calls to fence.check():
            feature_define_whitelist.add(r'AP_FENCE_ENABLED')
            # RPM not instantiated on Blimp or Rover:
            feature_define_whitelist.add(r'AP_RPM_ENABLED')
            feature_define_whitelist.add(r'AP_RPM_.*_ENABLED')
            # rangefinder init is not called:
            feature_define_whitelist.add(r'HAL_MSP_RANGEFINDER_ENABLED')
            # these guys don't instantiate anything which uses sd-card storage:
            feature_define_whitelist.add(r'AP_SDCARD_STORAGE_ENABLED')
            feature_define_whitelist.add(r'AP_RANGEFINDER_ENABLED')
            feature_define_whitelist.add(r'AP_RANGEFINDER_.*_ENABLED')

        if target.lower() in ["blimp", "antennatracker", "sub"]:
            # no OSD on Sub/blimp/tracker
            feature_define_whitelist.add(r'OSD_ENABLED')
            feature_define_whitelist.add(r'OSD_PARAM_ENABLED')
            # AP_OSD is not instantiated, , so no MSP backend:
            feature_define_whitelist.add(r'HAL_WITH_MSP_DISPLAYPORT')
            feature_define_whitelist.add(r'AP_MSP_INAV_FONTS_ENABLED')
            # camera instantiated in specific vehicles:
            feature_define_whitelist.add(r'AP_CAMERA_ENABLED')
            feature_define_whitelist.add(r'AP_CAMERA_.*_ENABLED')
            # button update is not called in these vehicles
            feature_define_whitelist.add(r'HAL_BUTTON_ENABLED')
            # precland not instantiated on these vehicles
            feature_define_whitelist.add(r'AC_PRECLAND_ENABLED')
            feature_define_whitelist.add(r'AC_PRECLAND_.*_ENABLED')
            # RSSI is not initialised - probably should be for some
            feature_define_whitelist.add(r'AP_RSSI_ENABLED')

        if target.lower() in ["antennatracker", "sub"]:
            # missing the init call to the relay library:
            feature_define_whitelist.add(r'AP_RELAY_ENABLED')
            feature_define_whitelist.add(r'AP_RC_CHANNEL_AUX_FUNCTION_STRINGS_ENABLED')

        if target.lower() in {"antennatracker", "blimp", "rover"}:
            # these don't instantiate terrain
            feature_define_whitelist.add('EK3_FEATURE_OPTFLOW_SRTM')

        if target.lower() not in ["AP_Periph"]:
            feature_define_whitelist.add(r'AP_PERIPH_.*')

        for some_re in feature_define_whitelist:
            if re.match(some_re, define):
                return True

    def assert_feature_in_code(self, defines, feature):
        # if the feature is truly disabled then extract_features.py
        # should say so:
        for target in self.build_targets:
            path = self.target_to_elf_path(target)
            extractor = extract_features.ExtractFeatures(path)
            (compiled_in_feature_defines, not_compiled_in_feature_defines) = extractor.extract()
            for define in defines:
                if not defines[define]:
                    continue
                if define in compiled_in_feature_defines:
                    continue
                error = f"feature gated by {define} not compiled into ({target}); extract_features.py bug?"
                if self.define_is_whitelisted_for_feature_in_code(target, define):
                    print("warn: " + error)
                    continue
                raise ValueError(error)

    def board(self):
        '''returns board to build for'''
        return self._board

    def parallel_dir(self):
        '''directory holding the parallel workers' build directories and logs'''
        return os.path.join("build", "test_build_options")

    def build_dir(self):
        '''directory waf builds into'''
        if self.worker_number is None:
            return "build"
        return os.path.join(self.parallel_dir(), "worker-%u" % self.worker_number)

    def extra_hwdef_filepath(self):
        if self.worker_number is None:
            return "/tmp/extra.hwdef"
        return os.path.join(self.parallel_dir(), "worker-%u-extra.hwdef" % self.worker_number)

    def jobs_per_worker(self):
        '''waf -j value for each build, or None for waf's default'''
        if self.jobs is None:
            return None
        if self.parallel is None:
            return self.jobs
        return max(1, math.ceil(self.jobs / self.parallel))

    def test_compile_with_defines(self, defines):
        extra_hwdef_filepath = self.extra_hwdef_filepath()
        self.write_defines_to_file(defines, extra_hwdef_filepath)
        if self.extra_hwdef is not None:
            content = open(self.extra_hwdef, "r").read()
            with open(extra_hwdef_filepath, "a") as f:
                f.write(content)
        extra_args = []
        if self.worker_number is not None:
            extra_args.extend(["--out", self.build_dir()])
        util.waf_configure(
            self.board(),
            extra_hwdef=extra_hwdef_filepath,
            extra_args=extra_args,
        )
        build_args = []
        jobs = self.jobs_per_worker()
        if jobs is not None:
            build_args.extend(["-j", str(jobs)])
        for t in self.build_targets:
            try:
                util.run_cmd([util.relwaf(), t] + build_args)
            except Exception:
                print("Failed to build (%s) with things disabled" %
                      (t,))
                raise

    def target_to_path(self, target, extension=None):
        '''given a build target (e.g. copter), return expected path to .bin
        file for that target'''
        target_to_binpath = {
            "copter": "arducopter",
            "plane": "arduplane",
            "rover": "ardurover",
            "antennatracker": "antennatracker",
            "sub": "ardusub",
            "blimp": "blimp",
        }
        filename = target_to_binpath[target]
        if extension is not None:
            filename += "." + extension
        return os.path.join(self.build_dir(), self.board(), "bin", filename)

    def target_to_bin_path(self, target):
        '''given a build target (e.g. copter), return expected path to .bin
        file for that target'''
        return self.target_to_path(target, 'bin')

    def target_to_elf_path(self, target):
        '''given a build target (e.g. copter), return expected path to .elf
        file for that target'''
        return self.target_to_path(target)

    def find_build_sizes(self):
        '''returns a hash with size of all build targets'''
        ret = {}
        for target in self.build_targets:
            path = self.target_to_bin_path(target)
            ret[target] = os.path.getsize(path)
        return ret

    def csv_for_results(self, results):
        '''return a string with csv for results'''
        features = sorted(results.keys())
        all_vehicles = set()
        for feature in features:
            all_vehicles.update(list(results[feature].keys()))
        sorted_all_vehicles = sorted(list(all_vehicles))
        ret = ""
        ret += ",".join(["Feature"] + sorted_all_vehicles) + "\n"
        for feature in features:
            line = [feature]
            feature_results = results[feature]
            for vehicle in sorted_all_vehicles:
                bytes_delta = ""
                if vehicle in feature_results:
                    result = feature_results[vehicle]
                    bytes_delta = result.bytes_delta
                line.append(str(bytes_delta))
            ret += ",".join(line) + "\n"
        return ret

    def disable_in_turn_check_sizes(self, feature, current_sizes):
        if not self.do_step_disable_none:
            self.progress("disable-none skipped, size comparison not available")
            return
        if self.sizes_nothing_disabled is None:
            self.progress("disable-none failed, size comparison not available")
            return
        for (build, new_size) in current_sizes.items():
            old_size = self.sizes_nothing_disabled[build]
            self.progress("Disabling %s(%s) on %s saves %u bytes" %
                          (feature.label, feature.define, build, old_size - new_size))
            if feature.define not in self.results:
                self.results[feature.define] = {}
            self.results[feature.define][build] = TestBuildOptionsResult(feature.define, build, old_size - new_size)
            with open("/tmp/some.csv", "w") as f:
                f.write(self.csv_for_results(self.results))

    def disable_in_turn_progress_Path(self):
        return pathlib.Path("/tmp/run-disable-in-turn-progress")

    def disable_in_turn_features(self):
        '''returns a list of (count, feature) to be disabled in turn;
        count is the feature's position in the list of options'''
        resume_number = self.resume_number_from_progress_Path(self.disable_in_turn_progress_Path())
        options = self.get_build_options_from_ardupilot_tree()
        ret = []
        for (count, feature) in enumerate(sorted(options, key=lambda x : x.define), start=1):
            if resume_number is not None and count < resume_number:
                continue
            if self.match_glob is not None:
                if not fnmatch.fnmatch(feature.define, self.match_glob):
                    continue
            if feature.category == 'AP_Periph' and not self.board_object().is_ap_periph:
                continue
            if feature.define in self.must_have_defines_for_board(self._board):
                self.progress("Feature %s(%s) (%u/%u) is a MUST-HAVE" %
                              (feature.label, feature.define, count, len(options)))
                continue
            ret.append((count, feature))
        return ret

    def run_serial_build(self, name, function):
        '''calls function, which runs a build; returns True if it
        succeeded.  A failure is recorded and, unless we are to stop
        on failure, the run carries on'''
        try:
            function()
            return True
        except Exception as ex:
            if self.stop_on_failure:
                raise
            traceback.print_exc()
            self.progress("FAILED %s: %s" % (name, str(ex)))
            self.failures.append((name, str(ex)))
            return False

    def run_disable_in_turn(self):
        progress_file = self.disable_in_turn_progress_Path()
        options = self.get_build_options_from_ardupilot_tree()
        # the progress file must not move past a failed feature, so
        # that --resume tests it again:
        first_failure = None
        for (count, feature) in self.disable_in_turn_features():
            if first_failure is None:
                self.write_progress_Path(progress_file, count, len(options), feature.define)
            self.progress("Disabling feature %s(%s) (%u/%u)" %
                          (feature.label, feature.define, count, len(options)))
            if not self.run_serial_build(
                    "disable-in-turn %s" % feature.define,
                    lambda: self.test_disable_feature(feature, options)):
                if first_failure is None:
                    first_failure = count
                continue
            self.disable_in_turn_check_sizes(feature, self.find_build_sizes())

    def write_progress_Path(self, progress_file, count, total, define):
        progress_file.write_text(f"{count}/{total} {define}\n")

    def enable_in_turn_check_sizes(self, feature, current_sizes):
        if not self.do_step_disable_all:
            self.progress("disable-none skipped, size comparison not available")
            return
        if self.sizes_everything_disabled is None:
            self.progress("disable-all failed, size comparison not available")
            return
        for (build, new_size) in current_sizes.items():
            old_size = self.sizes_everything_disabled[build]
            self.progress("Enabling %s(%s) on %s costs %u bytes" %
                          (feature.label, feature.define, build, old_size - new_size))
            if feature.define not in self.enable_in_turn_results:
                self.enable_in_turn_results[feature.define] = {}
            self.enable_in_turn_results[feature.define][build] = TestBuildOptionsResult(feature.define, build, old_size - new_size)  # noqa
            with open("/tmp/enable-in-turn.csv", "w") as f:
                f.write(self.csv_for_results(self.enable_in_turn_results))

    def resume_number_from_progress_Path(self, progress_file):
        if not self.resume:
            return None
        try:
            content = progress_file.read_text().rstrip()
            m = re.match(r"(\d+)/\d+ \w+", content)
            if m is None:
                raise ValueError(f"{progress_file} not matched")
            return int(m.group(1))
        except FileNotFoundError:
            pass
        return None

    def enable_in_turn_progress_Path(self):
        return pathlib.Path("/tmp/run-enable-in-turn-progress")

    def enable_in_turn_features(self):
        '''returns a list of (count, feature) to be enabled in turn;
        count is the feature's position in the list of options'''
        resume_number = self.resume_number_from_progress_Path(self.enable_in_turn_progress_Path())
        options = self.get_build_options_from_ardupilot_tree()
        blacklisted_defines = {
            'AP_NETWORKING_CAN_MCAST_ENABLED': "can't enable this without one of native-ethernet or PPP backends, don't want either in our deps!",  # noqa:E501
            'AP_NETWORKING_CAPTURE_ENABLED': "can't enable this without one of native-ethernet or PPP backends, don't want either in our deps!",  # noqa:E501
            'AP_NETWORKING_ENABLED': "can't enable this without one of native-ethernet or PPP backends, don't want either in our deps!",  # noqa:E501
        }
        ret = []
        for (count, feature) in enumerate(options, start=1):
            if resume_number is not None and count < resume_number:
                continue
            if feature.define in blacklisted_defines:
                continue
            if feature.category == 'AP_Periph' and not self.board_object().is_ap_periph:
                continue
            if self.match_glob is not None:
                if not fnmatch.fnmatch(feature.define, self.match_glob):
                    continue
            ret.append((count, feature))
        return ret

    def run_enable_in_turn(self):
        progress_file = self.enable_in_turn_progress_Path()
        options = self.get_build_options_from_ardupilot_tree()
        # the progress file must not move past a failed feature, so
        # that --resume tests it again:
        first_failure = None
        for (count, feature) in self.enable_in_turn_features():
            self.progress("Enabling feature %s(%s) (%u/%u)" %
                          (feature.label, feature.define, count, len(options)))
            if first_failure is None:
                self.write_progress_Path(progress_file, count, len(options), feature.define)
            if not self.run_serial_build(
                    "enable-in-turn %s" % feature.define,
                    lambda: self.test_enable_feature(feature, options)):
                if first_failure is None:
                    first_failure = count
                continue
            self.enable_in_turn_check_sizes(feature, self.find_build_sizes())

    def get_option_by_label(self, label, options):
        for x in options:
            if x.label == label:
                return x
        raise ValueError("No such option (%s)" % label)

    def get_disable_all_defines(self):
        '''returns a hash of defines which turns all features off'''
        options = self.get_build_options_from_ardupilot_tree()
        defines = {}
        for feature in options:
            if self.match_glob is not None:
                if not fnmatch.fnmatch(feature.define, self.match_glob):
                    continue
            if feature.category == 'AP_Periph' and not self.board_object().is_ap_periph:
                continue
            defines[feature.define] = 0
        for define in self.must_have_defines_for_board(self._board):
            defines[define] = 1

        return defines

    def run_disable_all(self):
        defines = self.get_disable_all_defines()
        self.test_compile_with_defines(defines)
        self.sizes_everything_disabled = self.find_build_sizes()

    def run_disable_none(self):
        self.test_compile_with_defines({})
        self.sizes_nothing_disabled = self.find_build_sizes()

    def get_with_defaults_defines(self):
        options = self.get_build_options_from_ardupilot_tree()
        defines = {}
        for feature in options:
            if feature.category == 'AP_Periph' and not self.board_object().is_ap_periph:
                continue
            defines[feature.define] = feature.default
        return defines

    def run_with_defaults(self):
        self.test_compile_with_defines(self.get_with_defaults_defines())

    def check_deps_consistency(self):
        # self.progress("Checking deps consistency")
        options = self.get_build_options_from_ardupilot_tree()
        for feature in options:
            self.get_disable_defines(feature, options)

    def check_duplicate_labels(self):
        '''check that we do not have multiple features with same labels'''
        options = self.get_build_options_from_ardupilot_tree()
        seen_labels = {}
        for feature in options:
            if seen_labels.get(feature.label, None) is not None:
                raise ValueError("Duplicate entries found for label '%s'" % feature.label)
            seen_labels[feature.label] = True

    def do_emit_disable_all_defines(self):
        defines = tbo.get_disable_all_defines()
        for f in self.must_have_defines():
            defines[f] = 1
        tbo.write_defines_to_Path(defines, pathlib.Path("/dev/stdout"))
        sys.exit(0)

    def get_option_by_define(self, define, options):
        for x in options:
            if x.define == define:
                return x
        raise ValueError("No such option (%s)" % define)

    def run_build(self, step, define):
        '''runs the build (and feature checks) for one step, returning
        the sizes of the binaries built'''
        options = self.get_build_options_from_ardupilot_tree()
        if step == "with-defaults":
            self.test_compile_with_defines(self.get_with_defaults_defines())
        elif step == "disable-all":
            self.test_compile_with_defines(self.get_disable_all_defines())
        elif step == "disable-none":
            self.test_compile_with_defines({})
        elif step == "disable-in-turn":
            self.test_disable_feature(self.get_option_by_define(define, options), options)
        elif step == "enable-in-turn":
            self.test_enable_feature(self.get_option_by_define(define, options), options)
        else:
            raise ValueError("Unknown step (%s)" % step)
        return self.find_build_sizes()

    def build_log_path(self, step, define):
        name = step
        if define is not None:
            name += "-" + define
        return os.path.join(self.parallel_dir(), "logs", name + ".log")

    def parallel_worker_main(self, worker_number, task_queue, result_queue, stop_event):
        '''runs in a child process; takes builds from task_queue until
        it gets None, putting a result for each on result_queue'''
        self.worker_number = worker_number
        # we are leader of a process group holding this worker and the
        # builds it runs, so they can all be killed together.  Kill
        # them if our parent goes away, else they would carry on
        # working through the queue:
        try:
            os.setpgid(0, 0)
        except PermissionError:
            # our parent has already done it
            pass
        parent_pid = os.getppid()

        def watch_parent():
            while True:
                time.sleep(5)
                if os.getppid() != parent_pid:
                    os.killpg(0, signal.SIGTERM)

        threading.Thread(target=watch_parent, daemon=True).start()

        # each worker gets its own waf lock file so that the workers'
        # waf invocations each find their own build directory:
        os.environ["WAFLOCK"] = ".lock-waf_%s_test_build_options_%u" % (sys.platform, worker_number)
        # stop the workers' concurrent configures all rewriting the
        # source tree's .vscode/c_cpp_properties.json:
        os.environ["AP_NO_COMPILE_COMMANDS"] = "1"
        self.progress_stream = os.fdopen(os.dup(sys.stderr.fileno()), "w")
        # keep our output in order with that of the programs we run:
        sys.stdout.reconfigure(line_buffering=True)
        while True:
            task = task_queue.get()
            if task is None:
                break
            (step, count, define) = task
            if stop_event.is_set():
                result_queue.put((step, count, define, "skipped", None))
                continue
            # send everything this build prints to its own log file:
            log_path = self.build_log_path(step, define)
            self.progress("Starting %s (log: %s)" % (step if define is None else "%s %s" % (step, define), log_path))
            result_queue.put((step, count, define, "started", (worker_number, log_path)))
            sys.stdout.flush()
            sys.stderr.flush()
            fd = os.open(log_path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o644)
            os.dup2(fd, sys.stdout.fileno())
            os.dup2(fd, sys.stderr.fileno())
            os.close(fd)
            try:
                sizes = self.run_build(step, define)
                result_queue.put((step, count, define, "ok", sizes))
            except Exception as ex:  # noqa: BLE001 — any failure of this build is reported, not fatal to the worker
                traceback.print_exc()
                sys.stderr.flush()
                result_queue.put((step, count, define, "failed", (log_path, str(ex))))

    def build_log_stage(self, log_path):
        '''returns a description of how far through its build the log at
        log_path is; util.run_cmd logs a "Running:" line for the
        configure and for each target'''
        try:
            n_run = pathlib.Path(log_path).read_text().count("Running: (")
        except FileNotFoundError:
            n_run = 0
        if n_run <= 1:
            return "configure"
        n_targets = len(self.build_targets)
        n_built = min(n_run - 1, n_targets)
        return "%u/%u - %s" % (n_built, n_targets, self.build_targets[n_built-1])

    def format_duration(self, seconds):
        seconds = int(seconds)
        if seconds >= 3600:
            return "%uh%02um" % (seconds // 3600, (seconds % 3600) // 60)
        return "%um%02us" % (seconds // 60, seconds % 60)

    def parallel_eta(self, completion_times, n_remaining, n_workers):
        '''returns a rough estimate of the time left, as a string.
        completion_times holds the time each build finished'''
        # each worker's first build is from scratch and much slower
        # than the rest, so wait until they are done and then use the
        # rate at which the most recent builds finished:
        if len(completion_times) <= n_workers:
            return "unknown until each worker has finished a build"
        window = completion_times[-min(len(completion_times) - n_workers, 2 * n_workers):]
        if len(window) < 2 or window[-1] <= window[0]:
            return "unknown"
        rate = (len(window) - 1) / (window[-1] - window[0])
        return "~" + self.format_duration(n_remaining / rate)

    def write_parallel_status(self, tstart, n_tasks, n_workers, n_passed, n_failed, n_skipped, in_flight,
                              completion_times, finished=False):
        '''write a human-readable summary of the run's state; not used
        by --resume'''
        now = time.time()
        state = "finished" if finished else "running"
        summary = "%u passed, %u failed, %u not started; elapsed %s" % (
            n_passed, n_failed, n_skipped, self.format_duration(now - tstart))
        if not finished:
            n_remaining = n_tasks - n_passed - n_failed - n_skipped
            summary += ", ETA %s" % self.parallel_eta(completion_times, n_remaining, n_workers)
        lines = [
            "%s: %u builds, %u workers, -j%s each" % (state, n_tasks, n_workers, self.jobs_per_worker()),
            summary,
        ]
        for worker_number in sorted(in_flight.keys()):
            (name, started, log_path) = in_flight[worker_number]
            lines.append("worker-%-3u %-60s %8s  (%s)" % (
                worker_number,
                name,
                self.format_duration(now - started),
                self.build_log_stage(log_path)))
        path = os.path.join(self.parallel_dir(), "status.txt")
        tmp_path = path + ".tmp"
        pathlib.Path(tmp_path).write_text("\n".join(lines) + "\n")
        os.replace(tmp_path, path)

    def run_parallel(self):
        '''run all of the requested builds, self.parallel at a time'''
        options = self.get_build_options_from_ardupilot_tree()

        tasks = []
        if self.do_step_run_with_defaults:
            tasks.append(("with-defaults", None, None))
        if self.do_step_disable_all:
            tasks.append(("disable-all", None, None))
        if self.do_step_disable_none:
            tasks.append(("disable-none", None, None))

        # (count => define) of builds not yet done, for the progress
        # files used by --resume:
        pending = {}
        progress_Paths = {
            "disable-in-turn": self.disable_in_turn_progress_Path(),
            "enable-in-turn": self.enable_in_turn_progress_Path(),
        }
        steps = []
        if self.do_step_disable_in_turn:
            steps.append(("disable-in-turn", self.disable_in_turn_features()))
        if self.do_step_enable_in_turn:
            steps.append(("enable-in-turn", self.enable_in_turn_features()))
        for (step, features) in steps:
            pending[step] = {}
            for (count, feature) in features:
                tasks.append((step, count, feature.define))
                pending[step][count] = feature.define

        def update_progress_file(step):
            if not pending[step]:
                return
            count = min(pending[step].keys())
            self.write_progress_Path(progress_Paths[step], count, len(options), pending[step][count])

        for step in pending.keys():
            update_progress_file(step)

        os.makedirs(os.path.join(self.parallel_dir(), "logs"), exist_ok=True)

        # size comparisons waiting for the build they compare against:
        deferred = []

        # baseline builds which have failed; the size comparisons
        # against them are skipped rather than waited for:
        failed_baselines = set()

        def check_deferred_sizes():
            still_deferred = []
            for (d_step, d_define, d_sizes) in deferred:
                feature = self.get_option_by_define(d_define, options)
                if d_step == "disable-in-turn":
                    if (self.do_step_disable_none and self.sizes_nothing_disabled is None and
                            "disable-none" not in failed_baselines):
                        still_deferred.append((d_step, d_define, d_sizes))
                        continue
                    self.disable_in_turn_check_sizes(feature, d_sizes)
                else:
                    if (self.do_step_disable_all and self.sizes_everything_disabled is None and
                            "disable-all" not in failed_baselines):
                        still_deferred.append((d_step, d_define, d_sizes))
                        continue
                    self.enable_in_turn_check_sizes(feature, d_sizes)
            deferred[:] = still_deferred

        def handle_result(step, count, define, sizes):
            if step == "disable-all":
                self.sizes_everything_disabled = sizes
            elif step == "disable-none":
                self.sizes_nothing_disabled = sizes
            elif step in ["disable-in-turn", "enable-in-turn"]:
                deferred.append((step, define, sizes))
                pending[step].pop(count)
                update_progress_file(step)
            check_deferred_sizes()

        # fork so the workers inherit this object rather than it
        # being pickled; workers are started before anything is put
        # into the queues so no threads exist when we fork:
        ctx = multiprocessing.get_context("fork")
        task_queue = ctx.Queue()
        result_queue = ctx.Queue()
        stop_event = ctx.Event()
        n_workers = min(self.parallel, len(tasks))
        self.progress("Running %u builds, %u at a time, -j%s each" %
                      (len(tasks), n_workers, self.jobs_per_worker()))
        self.progress("To follow progress: watch -n 10 cat %s" %
                      os.path.abspath(os.path.join(self.parallel_dir(), "status.txt")))
        # anything buffered would be printed again by each worker:
        sys.stdout.flush()
        sys.stderr.flush()
        workers = []
        failures = []
        tstart = time.time()
        # when each build finished, for the ETA:
        completion_times = []
        # worker_number => (name, start time, log path) of running builds:
        in_flight = {}

        def write_status(finished=False):
            self.write_parallel_status(tstart, len(tasks), n_workers, done - skipped - len(failures), len(failures), skipped,
                                       in_flight, completion_times, finished=finished)

        done = 0
        skipped = 0
        completed = False
        try:
            # started inside the try so that if starting one fails
            # those already running are stopped:
            for worker_number in range(n_workers):
                p = ctx.Process(
                    target=self.parallel_worker_main,
                    args=(worker_number, task_queue, result_queue, stop_event),
                    name="test_build_options-worker-%u" % worker_number,
                )
                p.start()
                try:
                    # also done in the worker; whichever of us is first:
                    os.setpgid(p.pid, p.pid)
                except (PermissionError, ProcessLookupError):
                    pass
                workers.append(p)

            for task in tasks:
                task_queue.put(task)
            for _ in workers:
                task_queue.put(None)

            write_status()
            while done < len(tasks):
                try:
                    (step, count, define, status, data) = result_queue.get(timeout=10)
                except queue.Empty:
                    for p in workers:
                        if p.exitcode not in [None, 0]:
                            raise ValueError("%s died (exitcode=%s)" % (p.name, p.exitcode))
                    write_status()
                    continue
                name = step if define is None else "%s %s" % (step, define)
                if status == "started":
                    (worker_number, log_path) = data
                    in_flight[worker_number] = (name, time.time(), log_path)
                    write_status()
                    continue
                for (worker_number, (in_flight_name, started, log_path)) in list(in_flight.items()):
                    if in_flight_name == name:
                        del in_flight[worker_number]
                done += 1
                if status in ["ok", "failed"]:
                    completion_times.append(time.time())
                if status == "ok":
                    self.progress("Finished %s (%u/%u done, ETA %s)" %
                                  (name, done, len(tasks), self.parallel_eta(completion_times, len(tasks) - done, n_workers)))
                    handle_result(step, count, define, data)
                elif status == "failed":
                    (log_path, reason) = data
                    self.progress("FAILED %s: %s (log: %s)" % (name, reason, log_path))
                    failures.append((name, log_path))
                    if step in ["disable-all", "disable-none"]:
                        failed_baselines.add(step)
                        check_deferred_sizes()
                    if self.stop_on_failure:
                        # let the builds already running finish, start no more:
                        stop_event.set()
                elif status == "skipped":
                    skipped += 1
                write_status()
            write_status(finished=True)
            completed = True
        finally:
            stop_event.set()
            if not completed:
                # tasks nobody will now read may still be waiting to go
                # into the pipe; don't wait for them at exit:
                task_queue.cancel_join_thread()
                # e.g. KeyboardInterrupt; the workers are in their own
                # process groups so did not get the SIGINT:
                for p in workers:
                    try:
                        os.killpg(p.pid, signal.SIGTERM)
                    except ProcessLookupError:
                        pass
            for p in workers:
                p.join(timeout=60)
            for p in workers:
                if p.is_alive():
                    p.terminate()
                    p.join()

        if failures:
            for (name, log_path) in failures:
                print("Failed: %s (log: %s)" % (name, log_path))
                lines = pathlib.Path(log_path).read_text().splitlines()
                print("\n".join(lines[-50:]))
            raise ValueError("%u builds failed: %s" %
                             (len(failures), ", ".join([name for (name, log_path) in failures])))

    def run(self):
        self.check_deps_consistency()
        self.check_duplicate_labels()

        if self.emit_disable_all_defines:
            self.do_emit_disable_all_defines()
            sys.exit(0)

        if self.parallel is not None:
            self.run_parallel()
            return

        if self.do_step_run_with_defaults:
            self.progress("Running run-with-defaults step")
            self.run_serial_build("with-defaults", self.run_with_defaults)
        if self.do_step_disable_all:
            self.progress("Running disable-all step")
            self.run_serial_build("disable-all", self.run_disable_all)
        if self.do_step_disable_none:
            self.progress("Running disable-none step")
            self.run_serial_build("disable-none", self.run_disable_none)
        if self.do_step_disable_in_turn:
            self.progress("Running disable-in-turn step")
            self.run_disable_in_turn()
        if self.do_step_enable_in_turn:
            self.progress("Running enable-in-turn step")
            self.run_enable_in_turn()

        if self.failures:
            for (name, reason) in self.failures:
                print("Failed: %s: %s" % (name, reason))
            raise ValueError("%u builds failed: %s" %
                             (len(self.failures), ", ".join([name for (name, reason) in self.failures])))


if __name__ == '__main__':

    parser = optparse.OptionParser()
    parser.add_option("--define-match-glob",
                      type='string',
                      default=None,
                      help='feature define must match this glob to be tested')
    parser.add_option("--no-run-with-defaults",
                      action='store_true',
                      help='Do not run the run-with-defaults step')
    parser.add_option("--no-disable-all",
                      action='store_true',
                      help='Do not run the disable-all step')
    parser.add_option("--no-disable-none",
                      action='store_true',
                      help='Do not run the disable-none step')
    parser.add_option("--no-disable-in-turn",
                      action='store_true',
                      help='Do not run the disable-in-turn step')
    parser.add_option("--no-enable-in-turn",
                      action='store_true',
                      help='Do not run the enable-in-turn step')
    parser.add_option("--build-targets",
                      type='choice',
                      choices=TestBuildOptions.all_targets(),
                      action='append',
                      help='vehicle targets to build')
    parser.add_option("--extra-hwdef",
                      type='string',
                      default=None,
                      help="file containing extra hwdef information")
    parser.add_option("--board",
                      type='string',
                      default="CubeOrange",
                      help='board to build for')
    parser.add_option("--emit-disable-all-defines",
                      action='store_true',
                      help='emit defines used for disabling all features then exit')
    parser.add_option("--resume",
                      action='store_true',
                      help='resume from previous progress file')
    parser.add_option("--parallel",
                      type='int',
                      default=None,
                      help='run this many builds at once, each in its own build directory')
    parser.add_option("--jobs",
                      type='int',
                      default=None,
                      help='total number of build jobs; with --parallel this is divided between the builds (default: number of CPUs)')  # noqa:E501
    parser.add_option("--stop-on-failure",
                      action='store_true',
                      help='stop at the first failed build rather than testing everything and reporting all failures at the end')  # noqa:E501

    opts, args = parser.parse_args()

    if opts.parallel is not None and opts.parallel < 1:
        parser.error("--parallel must be 1 or more")

    tbo = TestBuildOptions(
        match_glob=opts.define_match_glob,
        do_step_disable_all=not opts.no_disable_all,
        do_step_disable_none=not opts.no_disable_none,
        do_step_disable_defaults=not opts.no_run_with_defaults,
        do_step_disable_in_turn=not opts.no_disable_in_turn,
        do_step_enable_in_turn=not opts.no_enable_in_turn,
        build_targets=opts.build_targets,
        board=opts.board,
        extra_hwdef=opts.extra_hwdef,
        emit_disable_all_defines=opts.emit_disable_all_defines,
        resume=opts.resume,
        parallel=opts.parallel,
        jobs=opts.jobs,
        stop_on_failure=opts.stop_on_failure,
    )

    tbo.run()
