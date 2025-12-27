#!/usr/bin/env python3

"""
script to determine what features have been built into an ArduPilot binary

AP_FLAKE8_CLEAN
"""
import argparse
import re
import sys
import build_options
from build_script_base import BuildScriptBase


if sys.version_info[0] < 3:
    running_python3 = False
else:
    running_python3 = True


class ExtractFeatures(BuildScriptBase):

    class FindString(object):
        def __init__(self, string):
            self.string = string

    def __init__(self, filename, nm="arm-none-eabi-nm", strings="strings"):
        super().__init__()
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
            ('AP_COMPASS_DRONECAN_HIRES_ENABLED', r'AP_Compass_DroneCAN::handle_magnetic_field_hires',),

            ('AP_AIS_ENABLED', 'AP_AIS::decode_position_report',),

            ('HAL_EFI_ENABLED', 'AP_EFI::AP_EFI',),
            ('AP_EFI_{type}_ENABLED', 'AP_EFI_(?P<type>.*)::update',),

            ('AP_EXTENDED_ESC_TELEM_ENABLED', r'AP_DroneCAN::handle_esc_ext_status\b',),

            ('AP_TEMPERATURE_SENSOR_ENABLED', 'AP_TemperatureSensor::AP_TemperatureSensor',),
            ('AP_TEMPERATURE_SENSOR_{type}_ENABLED', 'AP_TemperatureSensor_(?P<type>.*)::update',),

            ('AP_BEACON_ENABLED', 'AP_Beacon::AP_Beacon',),
            ('HAL_TORQEEDO_ENABLED', 'AP_Torqeedo::AP_Torqeedo'),

            ('HAL_NAVEKF3_AVAILABLE', 'NavEKF3::NavEKF3',),
            ('HAL_NAVEKF2_AVAILABLE', 'NavEKF2::NavEKF2',),
            ('AP_EXTERNAL_AHRS_ENABLED', r'AP_ExternalAHRS::init\b',),
            ('AP_EXTERNAL_AHRS_{type}_ENABLED', r'AP_ExternalAHRS_(?P<type>.*)::healthy\b',),
            ('HAL_INS_TEMPERATURE_CAL_ENABLE', 'AP_InertialSensor_TCal::Learn::save_calibration',),
            ('HAL_VISUALODOM_ENABLED', 'AP_VisualOdom::init',),

            ('AP_RANGEFINDER_ENABLED', 'RangeFinder::RangeFinder',),
            ('AP_RANGEFINDER_{type}_ENABLED', r'AP_RangeFinder_(?P<type>.*)::update\b',),
            ('AP_RANGEFINDER_{type}_ENABLED', r'AP_RangeFinder_(?P<type>.*)::get_reading\b',),
            ('AP_RANGEFINDER_{type}_ENABLED', r'AP_RangeFinder_(?P<type>.*)::model_dist_max_cm\b',),
            ('AP_RANGEFINDER_{type}_ENABLED', r'AP_RangeFinder_(?P<type>.*)::handle_frame\b',),
            ('AP_RANGEFINDER_LIGHTWARE_SERIAL_ENABLED', r'AP_RangeFinder_LightWareSerial::get_reading\b',),
            ('AP_RANGEFINDER_LWI2C_ENABLED', r'AP_RangeFinder_LightWareI2C::update\b',),
            ('AP_RANGEFINDER_MAXBOTIX_SERIAL_ENABLED', r'AP_RangeFinder_MaxsonarSerialLV::get_reading\b',),
            ('AP_RANGEFINDER_TRI2C_ENABLED', r'AP_RangeFinder_TeraRangerI2C::update\b',),
            ('AP_RANGEFINDER_JRE_SERIAL_ENABLED', r'AP_RangeFinder_JRE_Serial::get_reading\b',),
            ('AP_RANGEFINDER_RDS02UF_ENABLED', r'AP_RangeFinder_RDS02UF::get_reading\b',),
            ('AP_RANGEFINDER_HEXSOONRADAR_ENABLED', r'AP_RangeFinder_NRA24_CAN::handle_frame'),

            ('AP_GPS_NMEA_UNICORE_ENABLED', r'AP_GPS_NMEA::parse_agrica_field',),
            ('AP_GPS_{type}_ENABLED', r'AP_GPS_(?P<type>.*)::read\b',),

            ('AP_OPTICALFLOW_ENABLED', 'AP_OpticalFlow::AP_OpticalFlow',),
            ('AP_OPTICALFLOW_{type}_ENABLED', r'AP_OpticalFlow_(?P<type>.*)::update\b',),

            ('AP_BARO_{type}_ENABLED', r'AP_Baro_(?P<type>.*)::(_calculate|update)\b',),

            ('AP_MOTORS_TRI_ENABLED', 'AP_MotorsTri::set_frame_class_and_type'),
            ('AP_MOTORS_FRAME_{type}_ENABLED', r'AP_MotorsMatrix::setup_(?P<type>.*)_matrix\b',),

            ('HAL_MSP_ENABLED', r'AP_MSP::init\b',),
            ('HAL_MSP_{type}_ENABLED', r'AP_(?P<type>.*)_MSP::update\b',),
            ('HAL_MSP_{type}_ENABLED', r'AP_(?P<type>.*)_MSP::read\b',),
            ('HAL_WITH_MSP_DISPLAYPORT', r'AP_OSD_MSP_DisplayPort::init\b',),
            ('AP_MSP_INAV_FONTS_ENABLED', r'AP_OSD_MSP_DisplayPort::write_INAV\b',),


            ('AP_BATTERY_{type}_ENABLED', r'AP_BattMonitor_(?P<type>.*)::init\b',),
            ('AP_BATTERY_ESC_TELEM_OUTBOUND_ENABLED', r'AP_BattMonitor_Backend::update_esc_telem_outbound\b',),
            ('AP_BATTERY_WATT_MAX_ENABLED', 'Plane::throttle_watt_limiter',),

            ('HAL_MOUNT_ENABLED', 'AP_Mount::AP_Mount',),
            ('HAL_MOUNT_{type}_ENABLED', r'AP_Mount_(?P<type>.*)::update\b',),
            ('HAL_SOLO_GIMBAL_ENABLED', 'AP_Mount_SoloGimbal::init',),
            ('HAL_MOUNT_STORM32SERIAL_ENABLED', 'AP_Mount_SToRM32_serial::update',),
            ('HAL_MOUNT_STORM32MAVLINK_ENABLED', 'AP_Mount_SToRM32::update',),

            ('HAL_SPEKTRUM_TELEM_ENABLED', r'AP::spektrum_telem',),
            ('HAL_{type}_TELEM_ENABLED', r'AP_(?P<type>.*)_Telem::init',),
            ('AP_{type}_TELEM_ENABLED', r'AP_(?P<type>.*)_Telem::init',),
            ('HAL_CRSF_TELEM_TEXT_SELECTION_ENABLED', 'AP_CRSF_Telem::calc_text_selection',),
            ('AP_CRSF_SCRIPTING_ENABLED', 'AP_CRSF_Telem::get_menu_event',),
            ('AP_LTM_TELEM_ENABLED', 'AP_LTM_Telem::init',),
            ('HAL_HIGH_LATENCY2_ENABLED', 'GCS_MAVLINK::handle_control_high_latency',),

            ('AP_FRSKY_TELEM_ENABLED', 'AP::frsky_telem',),
            ('AP_FRSKY_D_TELEM_ENABLED', 'AP_Frsky_D::send',),
            ('AP_FRSKY_SPORT_TELEM_ENABLED', 'AP_Frsky_SPort::send_sport_frame',),
            ('AP_FRSKY_SPORT_PASSTHROUGH_ENABLED', 'AP_Frsky_SPort_Passthrough::process_packet',),
            ('HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL', 'AP_Frsky_SPort_Passthrough::set_telem_data'),

            ('AP_IBUS_TELEM_ENABLED', 'AP_IBus_Telem::init',),

            ('MODE_AUTOLAND_ENABLED', 'ModeAutoLand::update'),
            ('MODE_{type}_ENABLED', r'Mode(?P<type>.+)::init',),
            ('MODE_GUIDED_NOGPS_ENABLED', r'ModeGuidedNoGPS::init',),

            ('AP_CAMERA_ENABLED', 'AP_Camera::var_info',),
            ('AP_CAMERA_{type}_ENABLED', 'AP_Camera_(?P<type>.*)::trigger_pic',),
            ('AP_CAMERA_SEND_FOV_STATUS_ENABLED', 'AP_Camera::send_camera_fov_status'),
            ('AP_CAMERA_SEND_THERMAL_RANGE_ENABLED', 'AP_Camera::send_camera_thermal_range'),
            ('AP_CAMERA_INFO_FROM_SCRIPT_ENABLED', 'AP_Camera_Backend::set_camera_information'),
            ('AP_CAMERA_RUNCAM_ENABLED', 'AP_RunCam::AP_RunCam',),

            ('HAL_PROXIMITY_ENABLED', 'AP_Proximity::AP_Proximity',),
            ('AP_PROXIMITY_{type}_ENABLED', 'AP_Proximity_(?P<type>.*)::update',),
            ('AP_PROXIMITY_CYGBOT_ENABLED', 'AP_Proximity_Cygbot_D1::update',),
            ('AP_PROXIMITY_LIGHTWARE_{type}_ENABLED', 'AP_Proximity_LightWare(?P<type>.*)::update',),
            ('AP_PROXIMITY_HEXSOONRADAR_ENABLED', 'AP_Proximity_MR72_CAN::update',),
            ('AP_PROXIMITY_MR72_ENABLED', 'AP_Proximity_MR72_CAN::update',),

            ('HAL_PARACHUTE_ENABLED', 'AP_Parachute::update',),
            ('AP_FENCE_ENABLED', r'AC_Fence::check\b',),
            ('HAL_RALLY_ENABLED', 'AP_Rally::find_nearest_rally_point',),
            ('AP_AVOIDANCE_ENABLED', 'AC_Avoid::AC_Avoid',),
            ('AP_OAPATHPLANNER_ENABLED', 'AP_OAPathPlanner::AP_OAPathPlanner',),
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
            ('AP_PLANE_GLIDER_PULLUP_ENABLED', 'GliderPullup::in_pullup',),
            ('QAUTOTUNE_ENABLED', 'ModeQAutotune::_enter',),
            ('HAL_SOARING_ENABLED', 'SoaringController::var_info',),
            ('HAL_LANDING_DEEPSTALL_ENABLED', r'AP_Landing_Deepstall::override_servos',),

            ('AP_GRIPPER_ENABLED', r'AP_Gripper::init\b',),
            ('HAL_SPRAYER_ENABLED', 'AC_Sprayer::AC_Sprayer',),
            ('AP_LANDINGGEAR_ENABLED', r'AP_LandingGear::init\b',),
            ('AP_WINCH_ENABLED', 'AP_Winch::AP_Winch',),
            ('AP_WINCH_{type}_ENABLED', r'AP_Winch_(?P<type>.*)::update\b',),
            ('AP_RELAY_ENABLED', 'AP_Relay::init',),
            ('AP_SERVORELAYEVENTS_ENABLED', 'AP_ServoRelayEvents::update_events',),

            ('AP_RCPROTOCOL_ENABLED', r'AP_RCProtocol::init\b',),
            ('AP_RCPROTOCOL_MAVLINK_RADIO_ENABLED', r'AP_RCProtocol_MAVLinkRadio::update_radio_rc_channels',),
            ('AP_RCPROTOCOL_{type}_ENABLED', r'AP_RCProtocol_(?P<type>.*)::_process_byte\b',),
            ('AP_RCPROTOCOL_{type}_ENABLED', r'AP_RCProtocol_(?P<type>.*)::process_pulse\b',),

            ('AP_SERVO_TELEM_ENABLED', r'AP_Servo_Telem::update\b',),
            ('AP_VOLZ_ENABLED', r'AP_Volz_Protocol::init\b',),
            ('AP_DRONECAN_VOLZ_FEEDBACK_ENABLED', r'AP_DroneCAN::handle_actuator_status_Volz\b',),
            ('AP_ROBOTISSERVO_ENABLED', r'AP_RobotisServo::init\b',),
            ('AP_FETTEC_ONEWIRE_ENABLED', r'AP_FETtecOneWire::init\b',),
            ('AP_SBUSOUTPUT_ENABLED', 'AP_SBusOut::sbus_format_frame',),
            ('AP_KDECAN_ENABLED', r'AP_KDECAN::update\b',),

            ('AP_RPM_ENABLED', 'AP_RPM::AP_RPM',),
            ('AP_RPM_{type}_ENABLED', r'AP_RPM_(?P<type>.*)::update',),

            ('AP_OPENDRONEID_ENABLED', 'AP_OpenDroneID::update',),

            ('GPS_MOVING_BASELINE', r'MovingBase::var_info',),
            ('AP_DRONECAN_SEND_GPS', r'AP_GPS_DroneCAN::instance_exists\b',),
            ('AP_GPS_BLENDED_ENABLED', r'AP_GPS::calc_blend_weights\b',),

            ('HAL_WITH_DSP', r'AP_HAL::DSP::find_peaks\b',),
            ('AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED', r'AP_InertialSensor::HarmonicNotch::update_params\b',),
            ('AP_INERTIALSENSOR_BATCHSAMPLER_ENABLED', r'AP_InertialSensor::BatchSampler::init'),
            ('AP_INERTIALSENSOR_FAST_SAMPLE_WINDOW_ENABLED', r'FastRateBuffer::get_next_gyro_sample\b',),
            ('HAL_GYROFFT_ENABLED', r'AP_GyroFFT::AP_GyroFFT\b',),
            ('HAL_DISPLAY_ENABLED', r'Display::init\b',),
            ('HAL_NMEA_OUTPUT_ENABLED', r'AP_NMEA_Output::update\b',),
            ('HAL_BARO_WIND_COMP_ENABLED', r'AP_Baro::wind_pressure_correction\b',),
            ('AP_BARO_THST_COMP_ENABLED', r'AP_Baro::thrust_pressure_correction\b',),
            ('AP_TEMPCALIBRATION_ENABLED', r'AP_TempCalibration::apply_calibration',),

            ('HAL_PICCOLO_CAN_ENABLE', r'AP_PiccoloCAN::update',),
            ('EK3_FEATURE_EXTERNAL_NAV', r'NavEKF3_core::CorrectExtNavVelForSensorOffset'),
            ('EK3_FEATURE_DRAG_FUSION', r'NavEKF3_core::FuseDragForces'),
            ('EK3_FEATURE_OPTFLOW_FUSION', r'NavEKF3_core::FuseOptFlow'),
            ('EK3_FEATURE_OPTFLOW_SRTM', r'NavEKF3_core::writeTerrainData'),

            ('AP_RC_CHANNEL_AUX_FUNCTION_STRINGS_ENABLED', r'RC_Channel::lookuptable',),
            ('AP_SCRIPTING_ENABLED', r'AP_Scripting::init',),
            ('AP_SCRIPTING_SERIALDEVICE_ENABLED', r'AP_Scripting_SerialDevice::init',),

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
            ('AP_ADSB_AVOIDANCE_ENABLED', 'AP_Avoidance::init'),

            ('AP_SDCARD_STORAGE_ENABLED', 'StorageAccess::attach_file'),
            ('AP_MAVLINK_AUTOPILOT_VERSION_REQUEST_ENABLED', 'GCS_MAVLINK::handle_send_autopilot_version'),
            ('AP_MAVLINK_MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES_ENABLED', 'GCS_MAVLINK::handle_command_request_autopilot_capabilities'),  # noqa
            ('AP_MAVLINK_MSG_RELAY_STATUS_ENABLED', 'GCS_MAVLINK::send_relay_status'),
            ('AP_MAVLINK_MSG_DEVICE_OP_ENABLED', 'GCS_MAVLINK::handle_device_op_write'),
            ('AP_MAVLINK_SERVO_RELAY_ENABLED', 'GCS_MAVLINK::handle_servorelay_message'),
            ('AP_MAVLINK_MSG_SERIAL_CONTROL_ENABLED', 'GCS_MAVLINK::handle_serial_control'),
            ('AP_MAVLINK_MSG_MISSION_REQUEST_ENABLED', r'GCS_MAVLINK::handle_mission_request\b'),
            ('AP_MAVLINK_MSG_RC_CHANNELS_RAW_ENABLED', r'GCS_MAVLINK::send_rc_channels_raw\b'),
            ('AP_MAVLINK_FTP_ENABLED', 'GCS_FTP::init'),
            ('AP_MAVLINK_MAV_CMD_SET_HAGL_ENABLED', 'Plane::handle_external_hagl'),
            ('AP_MAVLINK_MSG_VIDEO_STREAM_INFORMATION_ENABLED', 'AP_Camera::send_video_stream_information'),
            ('AP_MAVLINK_MSG_FLIGHT_INFORMATION_ENABLED', 'GCS_MAVLINK::send_flight_information'),
            ('AP_MAVLINK_MSG_RANGEFINDER_SENDING_ENABLED', r'GCS_MAVLINK::send_rangefinder'),
            ('AP_MAVLINK_SIGNING_ENABLED', r'GCS_MAVLINK::load_signing_key'),

            ('AP_DRONECAN_HIMARK_SERVO_SUPPORT', 'AP_DroneCAN::SRV_send_himark'),
            ('AP_DRONECAN_HOBBYWING_ESC_SUPPORT', 'AP_DroneCAN::hobbywing_ESC_update'),
            ('COMPASS_CAL_ENABLED', 'CompassCalibrator::stop'),
            ('AP_TUNING_ENABLED', 'AP_Tuning::check_input'),
            ('AP_DRONECAN_SERIAL_ENABLED', 'AP_DroneCAN_Serial::update'),
            ('AP_SERIALMANAGER_IMUOUT_ENABLED', 'AP_InertialSensor::send_uart_data'),
            ('AP_NETWORKING_ENABLED', 'AP_Networking::init'),
            ('AP_NETWORKING_BACKEND_PPP', 'AP_Networking_PPP::init'),
            ('AP_NETWORKING_CAN_MCAST_ENABLED', 'AP_Networking_CAN::start'),
            ('AP_NETWORKING_CAPTURE_ENABLED', 'AP_Networking_Backend::capture_pbuf'),

            ('FORCE_APJ_DEFAULT_PARAMETERS', 'AP_Param::param_defaults_data'),
            ('HAL_BUTTON_ENABLED', 'AP_Button::update'),
            ('HAL_LOGGING_ENABLED', 'AP_Logger::init'),
            ('AP_COMPASS_CALIBRATION_FIXED_YAW_ENABLED', 'Compass::mag_cal_fixed_yaw'),
            ('COMPASS_LEARN_ENABLED', 'CompassLearn::update'),
            ('AP_CUSTOMROTATIONS_ENABLED', 'AP_CustomRotations::init'),
            ('AP_OSD_LINK_STATS_EXTENSIONS_ENABLED', r'AP_OSD_Screen::draw_rc_tx_power'),
            ('HAL_ENABLE_DRONECAN_DRIVERS', r'AP_DroneCAN::init'),
            ('AP_BARO_PROBE_EXTERNAL_I2C_BUSES', r'AP_Baro::_probe_i2c_barometers'),
            ('AP_RSSI_ENABLED', r'AP_RSSI::init'),
            ('AP_FOLLOW_ENABLED', 'AP_Follow::AP_Follow'),

            ('AP_ROVER_ADVANCED_FAILSAFE_ENABLED', r'Rover::afs_fs_check'),
            ('AP_ROVER_AUTO_ARM_ONCE_ENABLED', r'Rover::handle_auto_arm_once'),
            ('AP_COPTER_ADVANCED_FAILSAFE_ENABLED', r'Copter::afs_fs_check'),
            ('AP_COPTER_AHRS_AUTO_TRIM_ENABLED', r'RC_Channels_Copter::auto_trim_run'),

            ('AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED', r'GCS_MAVLINK_Plane::handle_command_int_guided_slew_commands'),
            ('AP_SERIALMANAGER_REGISTER_ENABLED', r'AP_SerialManager::register_port'),
            ('AP_QUICKTUNE_ENABLED', r'AP_Quicktune::update'),
            ('AP_FILTER_ENABLED', r'AP_Filters::update'),
            ('AP_CAN_LOGGING_ENABLED', r'AP_CANManager::can_logging_callback'),
            ('AP_PLANE_SYSTEMID_ENABLED', r'AP_SystemID::start'),
            ('AP_DDS_ENABLED', r'AP_DDS_Client::start'),
            ('AP_RC_TRANSMITTER_TUNING_ENABLED',  r'Copter::tuning'),
            ('AP_CPU_IDLE_STATS_ENABLED', r'AP_BoardConfig::use_idle_stats'),

            ('AP_PERIPH_DEVICE_TEMPERATURE_ENABLED', r'AP_Periph_FW::temperature_sensor_update'),
            ('AP_PERIPH_MSP_ENABLED', r'AP_Periph_FW::msp_init'),
            ('AP_PERIPH_NOTIFY_ENABLED', r'AP_Periph_FW::handle_notify_state'),
            ('AP_PERIPH_SERIAL_OPTIONS_ENABLED', r'SerialOptions::init'),
            ('AP_PERIPH_BATTERY_ENABLED', r'AP_Periph_FW::can_battery_update'),
            ('AP_PERIPH_RELAY_ENABLED', r'AP_Periph_FW::handle_hardpoint_command'),
            ('AP_PERIPH_BATTERY_BALANCE_ENABLED', r'AP_Periph_FW::batt_balance_update'),
            ('AP_PERIPH_BATTERY_TAG_ENABLED', r'BatteryTag::update'),
            ('AP_PERIPH_BATTERY_BMS_ENABLED', r'BatteryBMS::update'),
            ('AP_PERIPH_PROXIMITY_ENABLED', r'AP_Periph_FW::can_proximity_update'),
            ('AP_PERIPH_GPS_ENABLED', r'AP_Periph_FW::can_gps_update'),
            ('AP_PERIPH_ADSB_ENABLED', r'AP_Periph_FW::adsb_update'),
            ('AP_PERIPH_MAG_ENABLED', r'AP_Periph_FW::can_mag_update'),
            ('AP_PERIPH_BARO_ENABLED', r'AP_Periph_FW::can_baro_update'),
            ('AP_PERIPH_RANGEFINDER_ENABLED', r'AP_Periph_FW::can_rangefinder_update'),
            ('AP_PERIPH_IMU_ENABLED', r'AP_Periph_FW::can_imu_update'),
            ('AP_PERIPH_RC_OUT_ENABLED', r'AP_Periph_FW::sim_update_actuator'),
            ('AP_PERIPH_EFI_ENABLED', r'AP_Periph_FW::can_efi_update'),
            ('AP_PERIPH_RCIN_ENABLED', r'AP_Periph_FW::rcin_update'),
            ('AP_PERIPH_RPM_ENABLED', r'AP_Periph_FW::rpm_sensor_send'),
            ('AP_PERIPH_AIRSPEED_ENABLED', r'AP_Periph_FW::can_airspeed_update'),

            ('AP_SCRIPTING_BINDING_VEHICLE_ENABLED', 'AP_Vehicle_index'),
            ('AP_SCRIPTING_BINDING_MOTORS_ENABLED', 'AP__motors___index'),
        ]

    def progress_prefix(self):
        """Return prefix for progress messages."""
        return 'EF'

    def validate_features_list(self):
        '''ensures that every define present in build_options.py could be
        found by in our features list'''
        # a list of problematic defines we don't have fixes for ATM:
        whitelist = frozenset([
            'HAL_PERIPH_SUPPORT_LONG_CAN_PRINTF',  # this define changes single method body, hard to detect?
            'AP_PLANE_BLACKBOX_LOGGING', # no visible signature
            'AC_POLYFENCE_CIRCLE_INT_SUPPORT_ENABLED',  # no visible signature
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
        ], show_output=False, show_command=False)
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
        ], show_output=False, show_command=False)
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
    parser.add_argument('--nm', type=str, default="arm-none-eabi-nm", help='nm binary to use.')
    args = parser.parse_args()
    # print(args.firmware_file, args.nm)

    ef = ExtractFeatures(args.firmware_file, args.nm)
    ef.run()
