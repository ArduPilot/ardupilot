#!/usr/bin/env python

"""
script to determine what features have been built into an ArduPilot binary

AP_FLAKE8_CLEAN
"""

import optparse
import os
import re
import string
import subprocess
import sys
import time
import build_options


if sys.version_info[0] < 3:
    running_python3 = False
else:
    running_python3 = True


class ExtractFeatures(object):

    def __init__(self, filename):
        self.filename = filename
        self.nm = 'arm-none-eabi-nm'

        # feature_name should match the equivalent feature in
        # build_options.py ('FEATURE_NAME', 'EXPECTED_SYMBOL').
        # EXPECTED_SYMBOL is a regular expression which will be matched
        # against "define" in build_options's feature list, and
        # FEATURE_NAME will have substitutions made from the match.
        # the substitutions will be upper-cased
        self.features = [
            ('AP_AIRSPEED_ENABLED', 'AP_Airspeed::AP_Airspeed',),
            ('AP_AIRSPEED_{type}_ENABLED', r'AP_Airspeed_(?P<type>.*)::init',),

            ('HAL_ADSB_ENABLED', 'AP_ADSB::AP_ADSB',),
            ('HAL_ADSB_{type}_ENABLED', r'AP_ADSB_(?P<type>.*)::update',),
            ('HAL_ADSB_UCP_ENABLED', 'AP_ADSB_uAvionix_UCP::update',),
            ('AP_AIS_ENABLED', 'AP_AIS::AP_AIS',),

            ('HAL_EFI_ENABLED', 'AP_EFI::AP_EFI',),
            ('BEACON_ENABLED', 'AP_Beacon::AP_Beacon',),
            ('HAL_TORQEEDO_ENABLED', 'AP_Torqeedo::AP_Torqeedo'),

            ('HAL_NAVEKF3_AVAILABLE', 'NavEKF3::NavEKF3',),
            ('HAL_NAVEKF2_AVAILABLE', 'NavEKF2::NavEKF2',),
            ('HAL_EXTERNAL_AHRS_ENABLED', r'AP_ExternalAHRS::init\b',),
            ('HAL_INS_TEMPERATURE_CAL_ENABLE', 'AP_InertialSensor::TCal::Learn::save_calibration',),
            ('HAL_VISUALODOM_ENABLED', 'AP_VisualOdom::init',),

            ('AP_RANGEFINDER_ENABLED', 'RangeFinder::RangeFinder',),
            ('AP_RANGEFINDER_{type}_ENABLED', r'AP_RangeFinder_(?P<type>.*)::update\b',),
            ('AP_RANGEFINDER_{type}_ENABLED', r'AP_RangeFinder_(?P<type>.*)::get_reading\b',),
            ('AP_RANGEFINDER_{type}_ENABLED', r'AP_RangeFinder_(?P<type>.*)::model_dist_max_cm\b',),
            ('AP_RANGEFINDER_LIGHTWARE_SERIAL_ENABLED', r'AP_RangeFinder_LightWareSerial::get_reading\b',),
            ('AP_RANGEFINDER_LWI2C_ENABLED', r'AP_RangeFinder_LightWareI2C::update\b',),
            ('AP_RANGEFINDER_MAXBOTIX_SERIAL_ENABLED', r'AP_RangeFinder_MaxsonarSerialLV::get_reading\b',),
            ('AP_RANGEFINDER_TRI2C_ENABLED', r'AP_RangeFinder_TeraRangerI2C::update\b',),

            ('AP_GPS_{type}_ENABLED', r'AP_GPS_(?P<type>.*)::read\b',),

            ('AP_OPTICALFLOW_ENABLED', 'AP_OpticalFlow::AP_OpticalFlow',),
            ('AP_OPTICALFLOW_{type}_ENABLED', r'AP_OpticalFlow_(?P<type>.*)::update\b',),

            ('AP_BARO_{type}_ENABLED', r'AP_Baro_(?P<type>.*)::update\b',),

            ('AP_MOTORS_FRAME_{type}_ENABLED', r'AP_MotorsMatrix::setup_(?P<type>.*)_matrix\b',),

            ('HAL_MSP_ENABLED', r'AP_MSP::init\b',),
            ('HAL_MSP_{type}_ENABLED', r'AP_(?P<type>.*)_MSP::update\b',),
            ('HAL_MSP_{type}_ENABLED', r'AP_(?P<type>.*)_MSP::read\b',),
            ('HAL_WITH_MSP_DISPLAYPORT', r'AP_OSD_MSP_DisplayPort::init\b',),


            ('AP_BATTMON_{type}_ENABLE', r'AP_BattMonitor_(?P<type>.*)::init\b',),
            ('HAL_BATTMON_{type}_ENABLED', r'AP_BattMonitor_(?P<type>.*)::init\b',),

            ('HAL_MOUNT_ENABLED', 'AP_Mount::AP_Mount',),
            ('HAL_MOUNT_{type}_ENABLED', r'AP_Mount_(?P<type>.*)::update\b',),
            ('HAL_SOLO_GIMBAL_ENABLED', 'AP_Mount_SoloGimbal::init',),
            ('HAL_MOUNT_STORM32SERIAL_ENABLED', 'AP_Mount_SToRM32_serial::init',),
            ('HAL_MOUNT_STORM32MAVLINK_ENABLED', 'AP_Mount_SToRM32::init',),

            ('HAL_{type}_TELEM_ENABLED', r'AP_(?P<type>.*)_Telem::init',),
            ('HAL_CRSF_TELEM_TEXT_SELECTION_ENABLED', 'AP_CRSF_Telem::calc_text_selection',),
            ('AP_LTM_TELEM_ENABLED', 'AP_LTM_Telem::init',),
            ('HAL_HIGH_LATENCY2_ENABLED', 'GCS_MAVLINK::handle_control_high_latency',),

            ('MODE_{type}_ENABLED', r'Mode(?P<type>.+)::init',),
            ('MODE_GUIDED_NOGPS_ENABLED', r'ModeGuidedNoGPS::init',),

            ('HAL_RUNCAM_ENABLED', 'AP_RunCam::AP_RunCam',),

            ('HAL_PARACHUTE_ENABLED', 'AP_Parachute::update',),
            ('AP_FENCE_ENABLED', r'AC_Fence::check\b',),
            ('HAL_PROXIMITY_ENABLED', 'AP_Proximity::AP_Proximity',),
            ('AC_AVOID_ENABLED', 'AC_Avoid::AC_Avoid',),
            ('AC_OAPATHPLANNER_ENABLED', 'AP_OAPathPlanner::AP_OAPathPlanner',),

            ('AP_ICENGINE_ENABLED', 'AP_ICEngine::AP_ICEngine',),
            ('HAL_EFI_ENABLED', 'AP_RPM_EFI::AP_RPM_EFI',),
            ('HAL_EFI_NWPWU_ENABLED', r'AP_EFI_NWPMU::update\b',),
            ('HAL_GENERATOR_ENABLED', 'AP_Generator::AP_Generator',),

            ('OSD_ENABLED', 'AP_OSD::AP_OSD',),
            ('HAL_PLUSCODE_ENABLE', 'AP_OSD_Screen::draw_pluscode',),
            ('OSD_PARAM_ENABLED', 'AP_OSD_ParamScreen::AP_OSD_ParamScreen',),
            ('HAL_OSD_SIDEBAR_ENABLE', 'AP_OSD_Screen::draw_sidebars',),

            ('HAL_SMARTAUDIO_ENABLED', 'AP_SmartAudio::AP_SmartAudio',),
            ('AP_TRAMP_ENABLED', 'AP_Tramp::AP_Tramp',),

            ('HAL_QUADPLANE_ENABLED', 'QuadPlane::QuadPlane',),
            ('HAL_SOARING_ENABLED', 'SoaringController::var_info',),
            ('HAL_LANDING_DEEPSTALL_ENABLED', r'AP_Landing_Deepstall::terminate\b',),

            ('GRIPPER_ENABLED', r'AP_Gripper::init\b',),
            ('HAL_SPRAYER_ENABLED', 'AC_Sprayer::AC_Sprayer',),
            ('LANDING_GEAR_ENABLED', r'AP_LandingGear::init\b',),
            ('WINCH_ENABLED', 'AP_Winch::AP_Winch',),

            ('AP_VOLZ_ENABLED', r'AP_Volz_Protocol::init\b',),
            ('AP_ROBOTISSERVO_ENABLED', r'AP_RobotisServo::init\b',),
            ('AP_FETTEC_ONEWIRE_ENABLED', r'AP_FETtecOneWire::init\b',),

            ('RPM_ENABLED', 'AP_RPM::AP_RPM',),

            ('GPS_MOVING_BASELINE', r'AP_GPS_Backend::calculate_moving_base_yaw\b',),

            ('HAL_WITH_DSP', r'AP_HAL::DSP::find_peaks\b',),
            ('HAL_DISPLAY_ENABLED', r'Display::init\b',),
            ('HAL_NMEA_OUTPUT_ENABLED', r'AP_NMEA_Output::update\b',),
            ('HAL_BARO_WIND_COMP_ENABLED', r'AP_Baro::wind_pressure_correction\b',),

            ('HAL_PICCOLO_CAN_ENABLE', r'AP_PiccoloCAN::update',),
            ('EK3_FEATURE_EXTERNAL_NAV', r'NavEKF3::writeExtNavVelData'),
        ]

    def progress(self, string):
        '''pretty-print progress'''
        print("EF: %s" % string)

    def run_program(self, prefix, cmd_list, show_output=True, env=None):
        '''swiped from build_binaries.py'''
        if show_output:
            self.progress("Running (%s)" % " ".join(cmd_list))
        p = subprocess.Popen(cmd_list, bufsize=1, stdin=None,
                             stdout=subprocess.PIPE, close_fds=True,
                             stderr=subprocess.STDOUT, env=env)
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
            if running_python3:
                x = bytearray(x)
                x = filter(lambda x : chr(x) in string.printable, x)
                x = "".join([chr(c) for c in x])
            output += x
            x = x.rstrip()
            if show_output:
                print("%s: %s" % (prefix, x))
        (_, status) = returncode
        if status != 0 and show_output:
            self.progress("Process failed (%s)" %
                          str(returncode))
            raise subprocess.CalledProcessError(
                returncode, cmd_list)
        return output

    class Symbols(object):
        def __init__(self):
            self.symbols = dict()
            self.symbols_without_arguments = dict()

        def add(self, key, attributes):
            self.symbols[key] = attributes

            # also keep around the same symbol name without arguments.
            # if the key is already present then the attributes become
            # None as there are multiple possible answers...
            m = re.match("^([^(]+).*", key)
            if m is None:
                extracted_symbol_name = key
            else:
                extracted_symbol_name = m.group(1)
            # print("Adding (%s)" % str(extracted_symbol_name))
            if extracted_symbol_name in self.symbols_without_arguments:
                self.symbols_without_arguments[extracted_symbol_name] = None
            else:
                self.symbols_without_arguments[extracted_symbol_name] = attributes

        def dict_for_symbol(self, symbol):
            if '(' not in symbol:
                some_dict = self.symbols_without_arguments
            else:
                some_dict = self.symbols
            return some_dict

    def extract_symbols_from_elf(self, filename):
        '''parses ELF in filename, returns dict of symbols=>attributes'''
        text_output = self.run_program('EF', [
            self.nm,
            '--demangle',
            '--print-size',
            filename
        ], show_output=False)
        ret = ExtractFeatures.Symbols()
        for line in text_output.split("\n"):
            m = re.match("^([^ ]+) ([^ ]+) ([^ ]) (.*)", line.rstrip())
            if m is None:
                m = re.match("^([^ ]+) ([^ ]) (.*)", line.rstrip())
                if m is None:
                    # raise ValueError("Did not match (%s)" % line)
                    # e.g. Did not match (         U _errno)
                    continue
                (offset, symbol_type, symbol_name) = m.groups()
                size = "0"
            else:
                (offset, size, symbol_type, symbol_name) = m.groups()
            size = int(size, 16)
            # print("symbol (%s) size %u" % (str(symbol_name), size))
            ret.add(symbol_name, {
                "size": size,
            })

        return ret

    def run(self):

        build_options_defines = set([x.define for x in build_options.BUILD_OPTIONS])

        symbols = self.extract_symbols_from_elf(filename)

        remaining_build_options_defines = build_options_defines
        for (feature_define, symbol) in self.features:
            some_dict = symbols.dict_for_symbol(symbol)
            # look for symbols without arguments
            # print("Looking for (%s)" % str(name))
            for s in some_dict.keys():
                m = re.match(symbol, s)
                # print("matching %s with %s" % (symbol, s))
                if m is None:
                    continue
                d = m.groupdict()
                for key in d.keys():
                    d[key] = d[key].upper()
                # filter to just the defines present in
                # build_options.py - otherwise we end up with (e.g.)
                # AP_AIRSPEED_BACKEND_ENABLED, even 'though that
                # doesn't exist in the ArduPilot codebase.
                some_define = feature_define.format(**d)
                if some_define not in build_options_defines:
                    continue
                print(some_define)
                remaining_build_options_defines.discard(some_define)
        for remaining in sorted(remaining_build_options_defines):
            print("!" + remaining)


if __name__ == '__main__':

    parser = optparse.OptionParser("extract_features.py FILENAME")

    cmd_opts, cmd_args = parser.parse_args()

    if len(cmd_args) < 1:
        parser.print_help()
        sys.exit(1)

    filename = cmd_args[0]

    ef = ExtractFeatures(filename)
    ef.run()
