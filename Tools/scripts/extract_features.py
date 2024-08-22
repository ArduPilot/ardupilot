#!/usr/bin/env python

"""
script to determine what features have been built into an ArduPilot binary

AP_FLAKE8_CLEAN
"""
import argparse
import os
import re
import string
import subprocess
import sys
import time
import build_options
import select


if sys.version_info[0] < 3:
    running_python3 = False
else:
    running_python3 = True


class ExtractFeatures(object):

    class FindString(object):
        def __init__(self, string):
            self.string = string

    def __init__(self, filename, nm="arm-none-eabi-nm", strings="strings"):
        self.filename = filename
        self.nm = nm
        self.strings = strings

        # feature_name should match the equivalent feature in
        # build_options.py ('FEATURE_NAME', 'EXPECTED_SYMBOL').
        # EXPECTED_SYMBOL is a regular expression which will be matched
        # against "define" in build_options's feature list, and
        # FEATURE_NAME will have substitutions made from the match.
        # the substitutions will be upper-cased
        self.features = [
            ('AP_ADVANCEDFAILSAFE_ENABLED', r'AP_AdvancedFailsafe::heartbeat\b',),
            ('AP_BOOTLOADER_FLASHING_ENABLED', 'ChibiOS::Util::flash_bootloader',),
            ('AP_AIRSPEED_ENABLED', 'AP_Airspeed::AP_Airspeed',),
            ('AP_AIRSPEED_{type}_ENABLED', r'AP_Airspeed_(?P<type>.*)::init',),

            ('AC_PRECLAND_ENABLED', 'AC_PrecLand::AC_PrecLand',),
            ('AC_PRECLAND_ENABLED', 'AC_PrecLand::AC_PrecLand',),
            ('AC_PRECLAND_{type}_ENABLED', 'AC_PrecLand_(?P<type>.*)::update',),

            ('HAL_ADSB_ENABLED', 'AP_ADSB::AP_ADSB',),
            ('HAL_ADSB_{type}_ENABLED', r'AP_ADSB_(?P<type>.*)::update',),
            ('HAL_ADSB_UCP_ENABLED', 'AP_ADSB_uAvionix_UCP::update',),

            ('AP_COMPASS_{type}_ENABLED', r'AP_Compass_(?P<type>.*)::read\b',),
            ('AP_COMPASS_ICM20948_ENABLED', r'AP_Compass_AK09916::probe_ICM20948',),

            ('AP_AIS_ENABLED', 'AP_AIS::AP_AIS',),

            ('HAL_EFI_ENABLED', 'AP_EFI::AP_EFI',),
            ('AP_EFI_{type}_ENABLED', 'AP_EFI_(?P<type>.*)::update',),

            ('AP_TEMPERATURE_SENSOR_ENABLED', 'AP_TemperatureSensor::AP_TemperatureSensor',),
            ('AP_TEMPERATURE_SENSOR_{type}_ENABLED', 'AP_TemperatureSensor_(?P<type>.*)::update',),

            ('AP_BEACON_ENABLED', 'AP_Beacon::AP_Beacon',),
            ('HAL_TORQEEDO_ENABLED', 'AP_Torqeedo::AP_Torqeedo'),

            ('HAL_NAVEKF3_AVAILABLE', 'NavEKF3::NavEKF3',),
            ('HAL_NAVEKF2_AVAILABLE', 'NavEKF2::NavEKF2',),
            ('HAL_EXTERNAL_AHRS_ENABLED', r'AP_ExternalAHRS::init\b',),
            ('AP_EXTERNAL_AHRS_{type}_ENABLED', r'AP_ExternalAHRS_{type}::healthy\b',),
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
            ('AP_RANGEFINDER_JRE_SERIAL_ENABLED', r'AP_RangeFinder_JRE_Serial::get_reading\b',),

            ('AP_GPS_{type}_ENABLED', r'AP_GPS_(?P<type>.*)::read\b',),

            ('AP_OPTICALFLOW_ENABLED', 'AP_OpticalFlow::AP_OpticalFlow',),
            ('AP_OPTICALFLOW_{type}_ENABLED', r'AP_OpticalFlow_(?P<type>.*)::update\b',),

            ('AP_BARO_{type}_ENABLED', r'AP_Baro_(?P<type>.*)::update\b',),

            ('AP_MOTORS_FRAME_{type}_ENABLED', r'AP_MotorsMatrix::setup_(?P<type>.*)_matrix\b',),

            ('HAL_MSP_ENABLED', r'AP_MSP::init\b',),
            ('HAL_MSP_{type}_ENABLED', r'AP_(?P<type>.*)_MSP::update\b',),
            ('HAL_MSP_{type}_ENABLED', r'AP_(?P<type>.*)_MSP::read\b',),
            ('HAL_WITH_MSP_DISPLAYPORT', r'AP_OSD_MSP_DisplayPort::init\b',),


            ('AP_BATTERY_{type}_ENABLED', r'AP_BattMonitor_(?P<type>.*)::init\b',),
            ('AP_BATTERY_ESC_TELEM_OUTBOUND_ENABLED', r'AP_BattMonitor_Backend::update_esc_telem_outbound\b',),
            ('AP_BATTERY_WATT_MAX_ENABLED', 'AP_BattMonitor_Params::_watt_max',),

            ('HAL_MOUNT_ENABLED', 'AP_Mount::AP_Mount',),
            ('HAL_MOUNT_{type}_ENABLED', r'AP_Mount_(?P<type>.*)::update\b',),
            ('HAL_SOLO_GIMBAL_ENABLED', 'AP_Mount_SoloGimbal::init',),
            ('HAL_MOUNT_STORM32SERIAL_ENABLED', 'AP_Mount_SToRM32_serial::init',),
            ('HAL_MOUNT_STORM32MAVLINK_ENABLED', 'AP_Mount_SToRM32::init',),

            ('HAL_{type}_TELEM_ENABLED', r'AP_(?P<type>.*)_Telem::init',),
            ('AP_{type}_TELEM_ENABLED', r'AP_(?P<type>.*)_Telem::init',),
            ('HAL_CRSF_TELEM_TEXT_SELECTION_ENABLED', 'AP_CRSF_Telem::calc_text_selection',),
            ('AP_LTM_TELEM_ENABLED', 'AP_LTM_Telem::init',),
            ('HAL_HIGH_LATENCY2_ENABLED', 'GCS_MAVLINK::handle_control_high_latency',),

            ('AP_FRSKY_TELEM_ENABLED', 'AP::frsky_telem',),
            ('AP_FRSKY_D_TELEM_ENABLED', 'AP_Frsky_D::send',),
            ('AP_FRSKY_SPORT_TELEM_ENABLED', 'AP_Frsky_SPort::send_sport_frame',),
            ('AP_FRSKY_SPORT_PASSTHROUGH_ENABLED', 'AP::frsky_passthrough_telem',),

            ('MODE_{type}_ENABLED', r'Mode(?P<type>.+)::init',),
            ('MODE_GUIDED_NOGPS_ENABLED', r'ModeGuidedNoGPS::init',),

            ('AP_CAMERA_ENABLED', 'AP_Camera::var_info',),
            ('AP_CAMERA_{type}_ENABLED', 'AP_Camera_(?P<type>.*)::trigger_pic',),
            ('AP_CAMERA_SEND_FOV_STATUS_ENABLED', 'AP_Camera::send_camera_fov_status'),
            ('HAL_RUNCAM_ENABLED', 'AP_RunCam::AP_RunCam',),

            ('HAL_PROXIMITY_ENABLED', 'AP_Proximity::AP_Proximity',),
            ('AP_PROXIMITY_{type}_ENABLED', 'AP_Proximity_(?P<type>.*)::update',),
            ('AP_PROXIMITY_CYGBOT_ENABLED', 'AP_Proximity_Cygbot_D1::update',),
            ('AP_PROXIMITY_LIGHTWARE_{type}_ENABLED', 'AP_Proximity_LightWare(?P<type>.*)::update',),

            ('HAL_PARACHUTE_ENABLED', 'AP_Parachute::update',),
            ('AP_FENCE_ENABLED', r'AC_Fence::check\b',),
            ('HAL_RALLY_ENABLED', r'AP_Rally::get_rally_max\b',),
            ('AC_AVOID_ENABLED', 'AC_Avoid::AC_Avoid',),
            ('AC_OAPATHPLANNER_ENABLED', 'AP_OAPathPlanner::AP_OAPathPlanner',),
            ('AC_PAYLOAD_PLACE_ENABLED', 'PayloadPlace::start_descent'),
            ('AP_MISSION_NAV_PAYLOAD_PLACE_ENABLED', ExtractFeatures.FindString('PayloadPlace')),
            ('AP_ICENGINE_ENABLED', 'AP_ICEngine::AP_ICEngine',),
            ('HAL_EFI_ENABLED', 'AP_RPM_EFI::AP_RPM_EFI',),
            ('AP_EFI_NWPWU_ENABLED', r'AP_EFI_NWPMU::update\b',),
            ('AP_EFI_CURRAWONG_ECU_ENABLED', r'AP_EFI_Currawong_ECU::update\b',),
            ('AP_EFI_SERIAL_HIRTH_ENABLED', r'AP_EFI_Serial_Hirth::update\b',),
            ('HAL_GENERATOR_ENABLED', 'AP_Generator::AP_Generator',),
            ('AP_GENERATOR_{type}_ENABLED', r'AP_Generator_(?P<type>.*)::init',),

            ('OSD_ENABLED', 'AP_OSD::update_osd',),
            ('HAL_PLUSCODE_ENABLE', 'AP_OSD_Screen::draw_pluscode',),
            ('OSD_PARAM_ENABLED', 'AP_OSD_ParamScreen::AP_OSD_ParamScreen',),
            ('HAL_OSD_SIDEBAR_ENABLE', 'AP_OSD_Screen::draw_sidebars',),

            ('AP_VIDEOTX_ENABLED', 'AP_VideoTX::AP_VideoTX',),
            ('AP_SMARTAUDIO_ENABLED', 'AP_SmartAudio::AP_SmartAudio',),
            ('AP_TRAMP_ENABLED', 'AP_Tramp::AP_Tramp',),

            ('AP_CHECK_FIRMWARE_ENABLED', 'AP_CheckFirmware::check_signed_bootloader',),

            ('HAL_QUADPLANE_ENABLED', 'QuadPlane::QuadPlane',),
            ('QAUTOTUNE_ENABLED', 'ModeQAutotune::_enter',),
            ('HAL_SOARING_ENABLED', 'SoaringController::var_info',),
            ('HAL_LANDING_DEEPSTALL_ENABLED', r'AP_Landing_Deepstall::terminate\b',),

            ('AP_GRIPPER_ENABLED', r'AP_Gripper::init\b',),
            ('HAL_SPRAYER_ENABLED', 'AC_Sprayer::AC_Sprayer',),
            ('AP_LANDINGGEAR_ENABLED', r'AP_LandingGear::init\b',),
            ('AP_WINCH_ENABLED', 'AP_Winch::AP_Winch',),
            ('AP_RELAY_ENABLED', 'AP_Relay::init',),
            ('AP_SERVORELAYEVENTS_ENABLED', 'AP_ServoRelayEvents::update_events',),

            ('AP_RCPROTOCOL_ENABLED', r'AP_RCProtocol::init\b',),
            ('AP_RCPROTOCOL_{type}_ENABLED', r'AP_RCProtocol_(?P<type>.*)::_process_byte\b',),
            ('AP_RCPROTOCOL_{type}_ENABLED', r'AP_RCProtocol_(?P<type>.*)::_process_pulse\b',),

            ('AP_VOLZ_ENABLED', r'AP_Volz_Protocol::init\b',),
            ('AP_DRONECAN_VOLZ_FEEDBACK_ENABLED', r'AP_DroneCAN::handle_actuator_status_Volz\b',),
            ('AP_ROBOTISSERVO_ENABLED', r'AP_RobotisServo::init\b',),
            ('AP_FETTEC_ONEWIRE_ENABLED', r'AP_FETtecOneWire::init\b',),
            ('AP_SBUSOUTPUT_ENABLED', 'AP_SBusOut::sbus_format_frame',),
            ('AP_KDECAN_ENABLED', r'AP_KDECAN::update\b',),

            ('AP_RPM_ENABLED', 'AP_RPM::AP_RPM',),
            ('AP_RPM_{type}_ENABLED', r'AP_RPM_(?P<type>.*)::update',),

            ('AP_OPENDRONEID_ENABLED', 'AP_OpenDroneID::update',),

            ('GPS_MOVING_BASELINE', r'AP_GPS_Backend::calculate_moving_base_yaw\b',),
            ('AP_DRONECAN_SEND_GPS', r'AP_GPS_DroneCAN::instance_exists\b',),

            ('HAL_WITH_DSP', r'AP_HAL::DSP::find_peaks\b',),
            ('HAL_GYROFFT_ENABLED', r'AP_GyroFFT::AP_GyroFFT\b',),
            ('HAL_DISPLAY_ENABLED', r'Display::init\b',),
            ('HAL_NMEA_OUTPUT_ENABLED', r'AP_NMEA_Output::update\b',),
            ('HAL_BARO_WIND_COMP_ENABLED', r'AP_Baro::wind_pressure_correction\b',),
            ('AP_TEMPCALIBRATION_ENABLED', r'AP_TempCalibration::apply_calibration',),

            ('HAL_PICCOLO_CAN_ENABLE', r'AP_PiccoloCAN::update',),
            ('EK3_FEATURE_EXTERNAL_NAV', r'NavEKF3::writeExtNavVelData'),
            ('EK3_FEATURE_DRAG_FUSION', r'NavEKF3_core::FuseDragForces'),

            ('AP_RC_CHANNEL_AUX_FUNCTION_STRINGS_ENABLED', r'RC_Channel::lookuptable',),
            ('AP_SCRIPTING_ENABLED', r'AP_Scripting::init',),

            ('AP_NOTIFY_TONEALARM_ENABLED', r'AP_ToneAlarm::init'),
            ('AP_NOTIFY_MAVLINK_PLAY_TUNE_SUPPORT_ENABLED', r'AP_Notify::handle_play_tune'),
            ('AP_NOTIFY_MAVLINK_LED_CONTROL_SUPPORT_ENABLED', r'AP_Notify::handle_led_control'),
            ('AP_NOTIFY_NCP5623_ENABLED', r'NCP5623::write'),
            ('AP_NOTIFY_PROFILED_ENABLED', r'ProfiLED::init_ports'),
            ('AP_NOTIFY_PROFILED_SPI_ENABLED', r'ProfiLED_SPI::rgb_set_id'),
            ('AP_NOTIFY_NEOPIXEL_ENABLED', r'NeoPixel::init_ports'),
            ('AP_FILESYSTEM_FORMAT_ENABLED', r'AP_Filesystem::format'),

            ('AP_FILESYSTEM_{type}_ENABLED', r'AP_Filesystem_(?P<type>.*)::open'),

            ('AP_INERTIALSENSOR_KILL_IMU_ENABLED', r'AP_InertialSensor::kill_imu'),
            ('AP_CRASHDUMP_ENABLED', 'CrashCatcher_DumpMemory'),
            ('AP_CAN_SLCAN_ENABLED', 'SLCAN::CANIface::var_info'),
            ('AC_POLYFENCE_FENCE_POINT_PROTOCOL_SUPPORT', 'AC_PolyFence_loader::handle_msg_fetch_fence_point'),
            ('AP_MAVLINK_RALLY_POINT_PROTOCOL_ENABLED', 'GCS_MAVLINK::handle_common_rally_message'),

            ('AP_SDCARD_STORAGE_ENABLED', 'StorageAccess::attach_file'),
            ('AP_MAVLINK_AUTOPILOT_VERSION_REQUEST_ENABLED', 'GCS_MAVLINK::handle_send_autopilot_version'),
            ('AP_MAVLINK_MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES_ENABLED', 'GCS_MAVLINK::handle_command_request_autopilot_capabilities'),  # noqa
            ('AP_MAVLINK_MSG_RELAY_STATUS_ENABLED', 'GCS_MAVLINK::send_relay_status'),
            ('AP_MAVLINK_BATTERY2_ENABLED', 'GCS_MAVLINK::send_battery2'),
            ('AP_MAVLINK_MSG_MOUNT_CONTROL_ENABLED', 'AP_Mount::handle_mount_control'),
            ('AP_MAVLINK_MSG_MOUNT_CONFIGURE_ENABLED', 'AP_Mount::handle_mount_configure'),
            ('AP_MAVLINK_MSG_DEVICE_OP_ENABLED', 'GCS_MAVLINK::handle_device_op_write'),
            ('AP_MAVLINK_SERVO_RELAY_ENABLED', 'GCS_MAVLINK::handle_servorelay_message'),
            ('AP_MAVLINK_MSG_SERIAL_CONTROL_ENABLED', 'GCS_MAVLINK::handle_serial_control'),
            ('AP_MAVLINK_MSG_MISSION_REQUEST_ENABLED', 'GCS_MAVLINK::handle_mission_request\b'),
            ('AP_DRONECAN_HIMARK_SERVO_SUPPORT', 'AP_DroneCAN::SRV_send_himark'),
            ('AP_DRONECAN_HOBBYWING_ESC_SUPPORT', 'AP_DroneCAN::hobbywing_ESC_update'),
            ('COMPASS_CAL_ENABLED', 'CompassCalibrator::stop'),
            ('AP_TUNING_ENABLED', 'AP_Tuning::check_input'),
            ('AP_DRONECAN_SERIAL_ENABLED', 'AP_DroneCAN_Serial::update'),
            ('AP_SERIALMANAGER_IMUOUT_ENABLED', 'AP_InertialSensor::send_uart_data'),
            ('AP_NETWORKING_BACKEND_PPP', 'AP_Networking_PPP::init'),
            ('FORCE_APJ_DEFAULT_PARAMETERS', 'AP_Param::param_defaults_data'),
            ('HAL_BUTTON_ENABLED', 'AP_Button::update'),
            ('HAL_LOGGING_ENABLED', 'AP_Logger::Init'),
            ('HAL_ENABLE_DRONECAN_DRIVERS', r'AP_DroneCAN::init'),
            ('AP_OSD_LINK_STATS_EXTENSIONS_ENABLED', r'AP_OSD_Screen::draw_rc_tx_power'),
        ]

    def progress(self, msg):
        """Pretty-print progress."""
        print("EF: %s" % msg)

    def validate_features_list(self):
        '''ensures that every define present in build_options.py could be
        found by in our features list'''
        # a list of problematic defines we don't have fixes for ATM:
        whitelist = frozenset([
            'HAL_PERIPH_SUPPORT_LONG_CAN_PRINTF',  # this define changes single method body, hard to detect?
            'AP_PLANE_BLACKBOX_LOGGING', # no visible signature
        ])
        for option in build_options.BUILD_OPTIONS:
            if option.define in whitelist:
                continue
            matched = False
            for (define, _) in self.features:
                # replace {type} with "match any number of word characters'''
                define_re = "^" + re.sub(r"{type}", "\\\\w+", define) + "$"
                # print("define re is (%s)" % define_re)
                if re.match(define_re, option.define):
                    matched = True
                    break
            if not matched:
                raise ValueError("feature (%s) is not matched in extract_features" %
                                 (option.define))

    def run_program(self, prefix, cmd_list, show_output=True, env=None):
        """Swiped from build_binaries.py."""
        if show_output:
            self.progress("Running (%s)" % " ".join(cmd_list))
        p = subprocess.Popen(
            cmd_list,
            stdin=None,
            stdout=subprocess.PIPE,
            close_fds=True,
            stderr=subprocess.PIPE,
            env=env)
        stderr = bytearray()
        output = ""
        while True:
            # read all of stderr:
            while True:
                (rin, _, _) = select.select([p.stderr.fileno()], [], [], 0)
                if p.stderr.fileno() not in rin:
                    break
                new = p.stderr.read()
                if len(new) == 0:
                    break
                stderr += new

            x = p.stdout.readline()
            if len(x) == 0:
                (rin, _, _) = select.select([p.stderr.fileno()], [], [], 0)
                if p.stderr.fileno() in rin:
                    stderr += p.stderr.read()

                returncode = os.waitpid(p.pid, 0)
                if returncode:
                    break
                    # select not available on Windows... probably...
                time.sleep(0.1)
                continue
            if running_python3:
                x = bytearray(x)
                x = filter(lambda x: chr(x) in string.printable, x)
                x = "".join([chr(c) for c in x])
            output += x
            x = x.rstrip()
            if show_output:
                print("%s: %s" % (prefix, x))
        (_, status) = returncode
        if status != 0:
            stderr = stderr.decode('utf-8')
            self.progress("Process failed (%s) (%s)" %
                          (str(returncode), stderr))
            raise subprocess.CalledProcessError(
                status, cmd_list, output=str(output), stderr=str(stderr))
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
        """Parses ELF in filename, returns dict of symbols=>attributes."""
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

    def extract_strings_from_elf(self, filename):
        """Runs strings on filename, returns as a list"""
        text_output = self.run_program('EF', [
            self.strings,
            filename
        ], show_output=False)
        return text_output.split("\n")

    def extract(self):
        '''returns two sets - compiled_in and not_compiled_in'''

        build_options_defines = set([x.define for x in build_options.BUILD_OPTIONS])

        symbols = self.extract_symbols_from_elf(self.filename)
        strings = self.extract_strings_from_elf(self.filename)

        remaining_build_options_defines = build_options_defines
        compiled_in_feature_defines = []
        for (feature_define, symbol) in self.features:
            if isinstance(symbol, ExtractFeatures.FindString):
                if symbol.string in strings:
                    some_define = feature_define
                    if some_define not in build_options_defines:
                        continue
                    compiled_in_feature_defines.append(some_define)
                    remaining_build_options_defines.discard(some_define)
            else:
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
                    compiled_in_feature_defines.append(some_define)
                    remaining_build_options_defines.discard(some_define)
        return (compiled_in_feature_defines, remaining_build_options_defines)

    def create_string(self):
        '''returns a string with compiled in and not compiled-in features'''

        (compiled_in_feature_defines, not_compiled_in_feature_defines) = self.extract()

        ret = ""

        combined = {}
        for define in sorted(compiled_in_feature_defines):
            combined[define] = True
        for define in sorted(not_compiled_in_feature_defines):
            combined[define] = False

        def squash_hal_to_ap(a):
            return re.sub("^HAL_", "AP_", a)

        for define in sorted(combined.keys(), key=squash_hal_to_ap):
            bang = ""
            if not combined[define]:
                bang = "!"
            ret += bang + define + "\n"
        return ret

    def run(self):
        self.validate_features_list()
        print(self.create_string())


if __name__ == '__main__':

    parser = argparse.ArgumentParser(prog='extract_features.py', description='Extract ArduPilot features from binaries')
    parser.add_argument('firmware_file', help='firmware binary')
    parser.add_argument('-nm', type=str, default="arm-none-eabi-nm", help='nm binary to use.')
    args = parser.parse_args()
    # print(args.firmware_file, args.nm)

    ef = ExtractFeatures(args.firmware_file, args.nm)
    ef.run()
