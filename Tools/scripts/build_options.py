'''
Provide structured data understood by the CustomBuild server app.py

AP_FLAKE8_CLEAN

'''


class Feature:
    '''defines a feature which can be built into the firmware, along with
    its dependencies'''
    def __init__(self,
                 category,
                 label,
                 define,
                 description,
                 default,
                 dependency):
        self.category = category
        self.label = label
        self.define = define
        self.description = description
        self.default = default
        self.dependency = dependency

    def config_option(self):
        '''the name of the configure option to be used by waf'''
        return "enable-" + self.label.replace(" ", "-")


# list of build options to offer

# Feature(category, label, macro, description, default, dependencies)
#
# category      : The category/group this feature belongs to
# label         : Globally unique identifier for the feature (max 30 chars)
# macro         : The macro which enables/disables the feature at compile-time.
#                 e.g., "AP_AIRSPEED_ENABLED".
# description   : User visible description shown on Custom build server.
# default       : Default state (1 = enabled, 0 = disabled)
# dependencies  : Comma-separated list of feature labels that this feature depends on
#                 (a single string with commas and no spaces). e.g., 'dependency1,dependency2'.
BUILD_OPTIONS = [
    Feature('AHRS', 'EKF3', 'HAL_NAVEKF3_AVAILABLE', 'Enable EKF3', 1, None),
    Feature('AHRS', 'EKF2', 'HAL_NAVEKF2_AVAILABLE', 'Enable EKF2', 0, None),
    Feature('AHRS', 'AHRS_EXT', 'AP_EXTERNAL_AHRS_ENABLED', 'Enable External AHRS', 0, None),
    Feature('AHRS', 'MicroStrain5', 'AP_EXTERNAL_AHRS_MICROSTRAIN5_ENABLED', 'Enable MICROSTRAIN 5-series external AHRS', 0, "AHRS_EXT"),  # noqa: E501
    Feature('AHRS', 'MicroStrain7', 'AP_EXTERNAL_AHRS_MICROSTRAIN7_ENABLED', 'Enable MICROSTRAIN 7-series external AHRS', 0, "AHRS_EXT"),  # noqa: E501
    Feature('AHRS', 'AHRS_EXT_VECTORNAV', 'AP_EXTERNAL_AHRS_VECTORNAV_ENABLED', 'Enable VectorNav external AHRS', 0, "AHRS_EXT"),  # noqa
    Feature('AHRS', 'InertialLabs', 'AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED', 'Enable InertialLabs external AHRS', 0, "AHRS_EXT"),  # noqa
    Feature('AHRS', 'SBG', 'AP_EXTERNAL_AHRS_SBG_ENABLED', 'Enable SBG external AHRS', 0, "AHRS_EXT"),
    Feature('AHRS', 'VISUALODOM', 'HAL_VISUALODOM_ENABLED', 'Enable Visual Odometry', 0, None),
    Feature('AHRS', 'EKF3_EXTNAV', 'EK3_FEATURE_EXTERNAL_NAV', 'Enable External navigation for EKF3', 0, 'EKF3'),
    Feature('AHRS', 'EKF3_WINDEST', 'EK3_FEATURE_DRAG_FUSION', 'Enable Wind estimation for EKF3', 0, 'EKF3'),
    Feature('AHRS', 'EKF3_OPTFLOW', 'EK3_FEATURE_OPTFLOW_FUSION', 'Enable OpticalFlow fusion for EKF3', 0, 'EKF3,OPTICALFLOW'),
    Feature('AHRS', 'EKF3_OPTFLOW_SRTM', 'EK3_FEATURE_OPTFLOW_SRTM', 'Enable OpticalFlow using SRTM for EKF3', 0, 'EKF3_OPTFLOW'),  # noqa: E501
    Feature('AHRS', 'BARO_WIND_COMP', 'HAL_BARO_WIND_COMP_ENABLED', 'Enable Baro wind compensation', 0, None),

    Feature('Safety', 'PARACHUTE', 'HAL_PARACHUTE_ENABLED', 'Enable Parachute', 0, None),
    Feature('Safety', 'FENCE', 'AP_FENCE_ENABLED', 'Enable Geofences', 2, None),
    Feature('Safety', 'RALLY', 'HAL_RALLY_ENABLED', 'Enable Rally points', 0, None),  # noqa
    Feature('Safety', 'AC_AVOID', 'AP_AVOIDANCE_ENABLED', 'Enable Object Avoidance', 0, 'FENCE'),
    Feature('Safety', 'AC_OAPATHPLANNER', 'AP_OAPATHPLANNER_ENABLED', 'Enable Object Avoidance Path Planner', 0, 'FENCE'),

    Feature('Battery', 'BATTERY_FUELFLOW', 'AP_BATTERY_FUELFLOW_ENABLED', 'Enable Fuel flow battery monitor', 0, None),
    Feature('Battery', 'BATTERY_FUELLEVEL_PWM', 'AP_BATTERY_FUELLEVEL_PWM_ENABLED', 'Enable PWM Fuel level battery monitor', 0, None),  # noqa: E501
    Feature('Battery', 'BATTERY_FUELLEVEL_ANALOG', 'AP_BATTERY_FUELLEVEL_ANALOG_ENABLED', 'Enable Analog Fuel level battry monitor', 0, None),  # noqa: E501
    Feature('Battery', 'BATTERY_SMBUS', 'AP_BATTERY_SMBUS_ENABLED', 'Enable SMBUS battery monitor', 0, None),
    Feature('Battery', 'BATTERY_INA2XX', 'AP_BATTERY_INA2XX_ENABLED', 'Enable INA2XX battery monitor', 0, None),
    Feature('Battery', 'BATTERY_INA3221', 'AP_BATTERY_INA3221_ENABLED', 'Enable INA3221 battery monitor', 0, None),
    Feature('Battery', 'BATTERY_SYNTHETIC_CURRENT', 'AP_BATTERY_SYNTHETIC_CURRENT_ENABLED', 'Enable Synthetic Current monitor', 0, None), # noqa: E501
    Feature('Battery', 'BATTERY_ESC_TELEM_OUT', 'AP_BATTERY_ESC_TELEM_OUTBOUND_ENABLED', 'Enable Ability to put battery monitor data into ESC telem stream', 0, None), # noqa: E501
    Feature('Battery', 'BATTERY_SUM', 'AP_BATTERY_SUM_ENABLED', 'Enable Synthetic sum-of-other-batteries backend', 0, None), # noqa: E501
    Feature('Battery', 'BATTERY_WATT_MAX', 'AP_BATTERY_WATT_MAX_ENABLED', 'Enable BATT_WATT_MAX parameter', 0, None), # noqa: E501


    Feature('Ident', 'ADSB', 'HAL_ADSB_ENABLED', 'Enable ADSB', 0, None),
    Feature('Ident', 'ADSB_SAGETECH', 'HAL_ADSB_SAGETECH_ENABLED', 'Enable Sagetech ADSB', 0, 'ADSB'),
    Feature('Ident', 'ADSB_SAGETECH_MXS', 'HAL_ADSB_SAGETECH_MXS_ENABLED', 'Enable Sagetech MXS ADSB', 0, 'ADSB'),
    Feature('Ident', 'ADSB_UAVIONIX', 'HAL_ADSB_UAVIONIX_MAVLINK_ENABLED', 'Enable UAvionix ADSB', 0, 'ADSB'),
    Feature('Ident', 'ADSB_UAVIONX_UCP', 'HAL_ADSB_UCP_ENABLED', 'Enable uAvionix UCP ADSB', 0 , 'ADSB'),
    Feature('Ident', 'AIS', 'AP_AIS_ENABLED', 'Enable AIS', 0, None),
    Feature('Ident', 'OpenDroneID', 'AP_OPENDRONEID_ENABLED', 'Enable OpenDroneID (Remote ID)', 0, None),

    Feature('Telemetry', 'CRSF', 'HAL_CRSF_TELEM_ENABLED', 'Enable CRSF telemetry', 0, 'FrSky SPort PassThrough,FrSky,FrSky SPort,RC_CRSF'),  # noqa
    Feature('Telemetry', 'CRSFText', 'HAL_CRSF_TELEM_TEXT_SELECTION_ENABLED', 'Enable CRSF text param selection', 0, 'CRSF,OSD_PARAM,FrSky SPort PassThrough,FrSky,FrSky SPort'),  # NOQA: E501
    Feature('Telemetry', 'CRSF Scripting', 'AP_CRSF_SCRIPTING_ENABLED', 'Enable CRSF Menu Scripting', 0, 'CRSFText,SCRIPTING,CRSF,OSD_PARAM,FrSky SPort PassThrough,FrSky,FrSky SPort'),  # noqa
    Feature('Telemetry', 'HOTT', 'HAL_HOTT_TELEM_ENABLED', 'Enable HOTT telemetry', 0, None),
    Feature('Telemetry', 'SPEKTRUM', 'HAL_SPEKTRUM_TELEM_ENABLED', 'Enable Spektrum telemetry', 0, None),
    Feature('Telemetry', 'LTM', 'AP_LTM_TELEM_ENABLED', 'Enable LTM telemetry', 0, None),
    Feature('Telemetry', 'AUX_FUNCTION_STRINGS', 'AP_RC_CHANNEL_AUX_FUNCTION_STRINGS_ENABLED', 'Enable Auxiliary function activation text messages', 0, None),  # noqa
    Feature('Telemetry', 'FrSky', 'AP_FRSKY_TELEM_ENABLED', 'Enable FrSky telemetry', 0, None),
    Feature('Telemetry', 'FrSky D', 'AP_FRSKY_D_TELEM_ENABLED', 'Enable FrSkyD telemetry', 0, 'FrSky'),
    Feature('Telemetry', 'FrSky SPort', 'AP_FRSKY_SPORT_TELEM_ENABLED', 'Enable FrSkySPort telemetry', 0, 'FrSky'),  # noqa
    Feature('Telemetry', 'FrSky SPort PassThrough', 'AP_FRSKY_SPORT_PASSTHROUGH_ENABLED', 'Enable FrSkySPort pass-through telemetry', 0, 'FrSky SPort,FrSky'),  # noqa
    Feature('Telemetry', 'Bidirectional FrSky Telemetry', 'HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL', 'Enable bidirectional FrSky telemetry', 0, 'FrSky SPort'),  # noqa
    Feature('Telemetry', 'GHST', 'AP_GHST_TELEM_ENABLED', 'Enable Ghost telemetry', 0, "RC_GHST"), # noqa
    Feature('Telemetry', 'i-BUS', 'AP_IBUS_TELEM_ENABLED', 'Enable i-BUS telemetry', 0, None),

    Feature('Notify', 'PLAY_TUNE', 'AP_NOTIFY_MAVLINK_PLAY_TUNE_SUPPORT_ENABLED', 'Enable MAVLink Play Tune command', 0, None),  # noqa
    Feature('Notify', 'TONEALARM', 'AP_NOTIFY_TONEALARM_ENABLED', 'Enable PWM tone alarm', 0, None),  # noqa
    Feature('Notify', 'LED_CONTROL', 'AP_NOTIFY_MAVLINK_LED_CONTROL_SUPPORT_ENABLED', 'Enable MAVLink LED control', 0, None),  # noqa
    Feature('Notify', 'NOTIFY_NCP5623', 'AP_NOTIFY_NCP5623_ENABLED', 'Enable NCP5623 LED', 0, None),  # noqa
    # Feature('Notify', 'NOTIFY_PCA9685', 'AP_NOTIFY_PCA9685_ENABLED', 'Enable PCA9685 LED', 0, None),  # noqa  linux-only
    Feature('Notify', 'NOTIFY_PROFILED', 'AP_NOTIFY_PROFILED_ENABLED', 'Enable ProfiLED', 0, None),  # noqa
    Feature('Notify', 'DISPLAY', 'HAL_DISPLAY_ENABLED', 'Enable I2C Displays', 0, None),
    Feature('Notify', 'NOTIFY_PROFILED_SPI', 'AP_NOTIFY_PROFILED_SPI_ENABLED', 'Enable ProfiLED (SPI)', 0, None),  # noqa
    Feature('Notify', 'NOTIFY_NEOPIXEL', 'AP_NOTIFY_NEOPIXEL_ENABLED', 'Enable NeoPixel LED strings', 0, None),  # noqa

    Feature('MSP', 'MSP', 'HAL_MSP_ENABLED', 'Enable MSP telemetry and MSP OSD', 0, 'OSD'),
    Feature('MSP', 'MSP_SENSORS', 'HAL_MSP_SENSORS_ENABLED', 'Enable MSP sensors', 0, 'MSP_GPS,MSP_BARO,MSP_COMPASS,MSP_AIRSPEED,MSP,MSP_OPTICALFLOW,MSP_RANGEFINDER,OSD'),   # NOQA: E501
    Feature('MSP', 'MSP_GPS', 'HAL_MSP_GPS_ENABLED', 'Enable MSP GPS', 0, 'MSP,OSD'),
    Feature('MSP', 'MSP_COMPASS', 'AP_COMPASS_MSP_ENABLED', 'Enable MSP compass', 0, 'MSP,OSD'),
    Feature('MSP', 'MSP_OPTICALFLOW', 'HAL_MSP_OPTICALFLOW_ENABLED', 'Enable MSP OpticalFlow', 0, 'MSP,OSD,OPTICALFLOW'), # also OPTFLOW dep   # NOQA: E501
    Feature('MSP', 'MSP_RANGEFINDER', 'HAL_MSP_RANGEFINDER_ENABLED', 'Enable MSP rangefinder', 0, 'MSP,OSD,RANGEFINDER'),
    Feature('MSP', 'MSP_DISPLAYPORT', 'HAL_WITH_MSP_DISPLAYPORT', 'Enable MSP DisplayPort OSD (aka CANVAS MODE)', 0, 'MSP,OSD'),   # NOQA: E501
    Feature('MSP', 'DISPLAYPORT_INAV_FONTS', 'AP_MSP_INAV_FONTS_ENABLED', 'Enable INAV Fonts Translation for DisplayPort', 0, 'MSP_DISPLAYPORT,MSP,OSD'),  # NOQA: E501

    Feature('ICE', 'ICE Engine', 'AP_ICENGINE_ENABLED', 'Enable Internal combustion engine support', 0, 'RPM'),
    Feature('ICE', 'EFI', 'HAL_EFI_ENABLED', 'Enable EFI monitoring', 0, None),
    Feature('ICE', 'EFI_MegaSquirt', 'AP_EFI_SERIAL_MS_ENABLED', 'Enable MegaSquirt EFI', 0, 'EFI'),
    Feature('ICE', 'EFI_Lutan', 'AP_EFI_SERIAL_LUTAN_ENABLED', 'Enable Lutan EFI', 0, 'EFI'),
    Feature('ICE', 'EFI_NMPWU', 'AP_EFI_NWPWU_ENABLED', 'Enable NMPMU EFI', 0, 'EFI'),
    Feature('ICE', 'EFI_CURRAWONGECU', 'AP_EFI_CURRAWONG_ECU_ENABLED', 'Enable Currawong ECU', 0, 'EFI'),
    Feature('ICE', 'EFI_HIRTH', 'AP_EFI_SERIAL_HIRTH_ENABLED', 'Enable Hirth ECU', 0, 'EFI'),
    Feature('ICE', 'EFI_DRONECAN', 'AP_EFI_DRONECAN_ENABLED', 'Enable DroneCAN EFI', 0, 'EFI,DroneCAN'),
    Feature('ICE', 'EFI_MAV', 'AP_EFI_MAV_ENABLED', 'Enable MAVLink EFI', 0, 'EFI'),

    Feature('Generator', 'GENERATOR', 'HAL_GENERATOR_ENABLED', 'Enable Generator', 0, None),
    Feature('Generator', 'GENERATOR_RICHENPOWER', 'AP_GENERATOR_RICHENPOWER_ENABLED', 'Enable Richenpower generator', 0, "GENERATOR"),  # noqa
    Feature('Generator', 'GENERATOR_IE2400', 'AP_GENERATOR_IE_2400_ENABLED', 'Enable IntelligentEnergy 2400', 0, "GENERATOR"),  # noqa
    Feature('Generator', 'GENERATOR_IE650', 'AP_GENERATOR_IE_650_800_ENABLED', 'Enable IntelligentEnergy 650 and 800', 0, "GENERATOR"),  # noqa
    Feature('Generator', 'GENERATOR_LOWEHEISER', 'AP_GENERATOR_LOWEHEISER_ENABLED', 'Enable Loweheiser generator', 0, "GENERATOR,EFI"),  # noqa
    Feature('Generator', 'GENERATOR_CORTEX', 'AP_GENERATOR_CORTEX_ENABLED', 'Enable Currawong Engineering Cortex generator', 0, "GENERATOR,PICCOLOCAN"),  # noqa

    Feature('OSD', 'OSD', 'OSD_ENABLED', 'Enable OSD', 0, None),
    Feature('OSD', 'PLUSCODE', 'HAL_PLUSCODE_ENABLE', 'Enable PlusCode', 0, 'OSD'),
    Feature('OSD', 'OSD_PARAM', 'OSD_PARAM_ENABLED', 'Enable OSD param', 0, None),
    Feature('OSD', 'OSD_SIDEBARS', 'HAL_OSD_SIDEBAR_ENABLE', 'Enable Scrolling sidebars', 0, 'OSD'),
    Feature('OSD', 'OSD_EXTENDED_LINK_STATS', 'AP_OSD_LINK_STATS_EXTENSIONS_ENABLED', 'Enable OSD panels with extended link stats data', 0, "OSD,RC_CRSF,MSP"),  # noqa

    Feature('VTX', 'VIDEO_TX', 'AP_VIDEOTX_ENABLED', 'Enable VideoTX control', 0, None),
    Feature('VTX', 'SMARTAUDIO', 'AP_SMARTAUDIO_ENABLED', 'Enable SmartAudio VTX contol', 0, "VIDEO_TX"),
    Feature('VTX', 'TRAMP', 'AP_TRAMP_ENABLED', 'Enable IRC Tramp VTX control', 0, "VIDEO_TX"),

    Feature('ESC', 'PICCOLOCAN', 'HAL_PICCOLO_CAN_ENABLE', 'Enable PiccoloCAN', 0, 'DroneCAN'),
    Feature('ESC', 'TORQEEDO', 'HAL_TORQEEDO_ENABLED', 'Enable Torqeedo motors', 0, None),

    Feature('ESC', 'ESC_EXTENDED_TELM', 'AP_EXTENDED_ESC_TELEM_ENABLED', 'Enable Extended ESC telemetry', 0, 'DroneCAN'),

    Feature('AP_Periph', 'LONG_TEXT', 'HAL_PERIPH_SUPPORT_LONG_CAN_PRINTF', 'Enable extended length text strings', 0, None),
    Feature('AP_Periph', 'PERIPH_DEVICE_TEMPERATURE', 'AP_PERIPH_DEVICE_TEMPERATURE_ENABLED', 'Emit DroneCAN Temperature Messages for AP_Temperature sensors', 0, 'TEMP'), # noqa
    Feature('AP_Periph', 'PERIPH_MSP', 'AP_PERIPH_MSP_ENABLED', 'Emit MSP protocol messages from AP_Periph', 0, 'MSP'),
    Feature('AP_Periph', 'PERIPH_NOTIFY', 'AP_PERIPH_NOTIFY_ENABLED', 'Handle DroneCAN messages for notification equipment (e.g. buzzers, lights etc.)', 0, None), # noqa
    Feature('AP_Periph', 'PERIPH_SERIAL_OPTIONS', 'AP_PERIPH_SERIAL_OPTIONS_ENABLED', 'Enable Serial Options on AP_Periph', 0, None), # noqa
    Feature('AP_Periph', 'PERIPH_BATTERY', 'AP_PERIPH_BATTERY_ENABLED', 'Emit DroneCAN battery info messages using AP_BattMonitor', 0, None), # noqa
    Feature('AP_Periph', 'PERIPH_RELAY', 'AP_PERIPH_RELAY_ENABLED', 'Handle DroneCAN hardpoint command', 0, 'RELAY'),
    Feature('AP_Periph', 'PERIPH_BATTERY_BALANCE', 'AP_PERIPH_BATTERY_BALANCE_ENABLED', 'Emit DroneCAN BatteryInfoAux messages for monitoring Battery Balance', 0, None), # noqa
    Feature('AP_Periph', 'PERIPH_BATTERY_TAG', 'AP_PERIPH_BATTERY_TAG_ENABLED', 'Emit DroneCAN BatteryTag messages', 0, None), # noqa
    Feature('AP_Periph', 'PERIPH_BATTERY_BMS', 'AP_PERIPH_BATTERY_BMS_ENABLED', 'Enable Battery BMS button and LED support', 0, None), # noqa
    Feature('AP_Periph', 'PERIPH_PROXIMITY', 'AP_PERIPH_PROXIMITY_ENABLED', 'Emit DroneCAN Proximity Messages for AP_Proximity sensors', 0, 'PROXIMITY'), # noqa
    Feature('AP_Periph', 'PERIPH_GPS', 'AP_PERIPH_GPS_ENABLED', 'Emit DroneCAN GNSS Messages for AP_GPS sensors', 0, None), # noqa
    Feature('AP_Periph', 'PERIPH_ADSB', 'AP_PERIPH_ADSB_ENABLED', 'Emit DroneCAN TrafficReport Messages for ADSB_VEHICLE MAVLink messages', 0, None), # noqa
    Feature('AP_Periph', 'PERIPH_MAG', 'AP_PERIPH_MAG_ENABLED', 'Emit DroneCAN MagneticFieldStrength Messages ', 0, None), # noqa
    Feature('AP_Periph', 'PERIPH_BARO', 'AP_PERIPH_BARO_ENABLED', 'Emit DroneCAN StaticTemperature and StaticPressure Messages for AP_Baro Sensors', 0, None), # noqa
    Feature('AP_Periph', 'PERIPH_RANGEFINDER', 'AP_PERIPH_RANGEFINDER_ENABLED', 'Emit DroneCAN range_sensor_Measurement Messages ', 0, 'RANGEFINDER'), # noqa
    Feature('AP_Periph', 'PERIPH_IMU', 'AP_PERIPH_IMU_ENABLED', 'Emit DroneCAN ahrs_RawIMU Messages ', 0, None),
    Feature('AP_Periph', 'PERIPH_RC_OUT', 'AP_PERIPH_RC_OUT_ENABLED', 'Emit DroneCAN actuator_Status Messages', 0, None),
    Feature('AP_Periph', 'PERIPH_EFI', 'AP_PERIPH_EFI_ENABLED', 'Emit DroneCAN ice_reciprocating_Status Messages', 0, 'EFI'),
    Feature('AP_Periph', 'PERIPH_RCIN', 'AP_PERIPH_RCIN_ENABLED', 'Emit DroneCAN RCInput Messages', 0, 'RC_Protocol'),
    Feature('AP_Periph', 'PERIPH_RPM', 'AP_PERIPH_RPM_ENABLED', 'Emit DroneCAN RPM messages for AP_RPM sensors', 0, 'RPM'),
    Feature('AP_Periph', 'PERIPH_AIRSPEED', 'AP_PERIPH_AIRSPEED_ENABLED', 'Emit DroneCAN air_data_RawAirData messages for AP_Airspeed sensors', 0, 'AIRSPEED'), # noqa

    Feature('Camera', 'Camera', 'AP_CAMERA_ENABLED', 'Enable Camera trigger', 0, None),
    Feature('Camera', 'Camera_MAVLink', 'AP_CAMERA_MAVLINK_ENABLED', 'Enable MAVLink camera ', 0, 'Camera'),
    Feature('Camera', 'Camera_MAVLinkCamV2', 'AP_CAMERA_MAVLINKCAMV2_ENABLED', 'Enable MAVLink CameraV2', 0, 'Camera'),
    Feature('Camera', 'Camera_Mount', 'AP_CAMERA_MOUNT_ENABLED', 'Enable Camera-in-Mount ', 0, 'Camera,MOUNT'),
    Feature('Camera', 'Camera_Relay', 'AP_CAMERA_RELAY_ENABLED', 'Enable Relay camera trigger', 0, 'Camera,RELAY'),
    Feature('Camera', 'Camera_Servo', 'AP_CAMERA_SERVO_ENABLED', 'Enable Servo camera trigger', 0, 'Camera'),
    Feature('Camera', 'Camera_Solo', 'AP_CAMERA_SOLOGIMBAL_ENABLED', 'Enable Solo gimbal', 0, 'Camera'),
    Feature('Camera', 'Camera_FOV_Status', 'AP_CAMERA_SEND_FOV_STATUS_ENABLED', 'Enable GCS camera FOV status', 0, 'Camera,MOUNT'),  # noqa: E501
    Feature('Camera', 'Camera_ThermalRange', 'AP_CAMERA_SEND_THERMAL_RANGE_ENABLED', 'Enable GCS camera thermal range', 0, 'Camera,MOUNT'),  # noqa: E501
    Feature('Camera', 'Camera_Info_From_Script', 'AP_CAMERA_INFO_FROM_SCRIPT_ENABLED', 'Enable Camera information messages via Lua script', 0, 'Camera,SCRIPTING'), # noqa

    Feature('Camera', 'RUNCAM', 'AP_CAMERA_RUNCAM_ENABLED', 'Enable RunCam control', 0, 'Camera'),

    Feature('Copter', 'MODE_ZIGZAG', 'MODE_ZIGZAG_ENABLED', 'Enable Mode ZigZag', 0, None),
    Feature('Copter', 'MODE_SYSTEMID', 'MODE_SYSTEMID_ENABLED', 'Enable Mode SystemID', 0, 'Logging'),
    Feature('Copter', 'MODE_SPORT', 'MODE_SPORT_ENABLED', 'Enable Mode Sport', 0, None),
    Feature('Copter', 'MODE_FOLLOW', 'MODE_FOLLOW_ENABLED', 'Enable Mode Follow', 0, 'AC_AVOID,AP_FOLLOW'),
    Feature('Copter', 'MODE_TURTLE', 'MODE_TURTLE_ENABLED', 'Enable Mode Turtle', 0, None),
    Feature('Copter', 'MODE_GUIDED_NOGPS', 'MODE_GUIDED_NOGPS_ENABLED', 'Enable Mode Guided NoGPS', 0, None),
    Feature('Copter', 'MODE_FLOWHOLD', 'MODE_FLOWHOLD_ENABLED', 'Enable Mode Flowhold', 0, "OPTICALFLOW"),
    Feature('Copter', 'MODE_FLIP', 'MODE_FLIP_ENABLED', 'Enable Mode Flip', 0, None),
    Feature('Copter', 'MODE_BRAKE', 'MODE_BRAKE_ENABLED', 'Enable Mode Brake', 0, None),
    Feature('Copter', 'COPTER_ADVANCED_FAILSAFE', 'AP_COPTER_ADVANCED_FAILSAFE_ENABLED', 'Enable Advanced Failsafe', 0, "ADVANCED_FAILSAFE"),  # NOQA: 501
    Feature('Copter', 'COPTER_AHRS_AUTO_TRIM', 'AP_COPTER_AHRS_AUTO_TRIM_ENABLED', 'Enable Copter AHRS AutoTrim', 0, None),  # noqa

    Feature('Rover', 'ROVER_ADVANCED_FAILSAFE', 'AP_ROVER_ADVANCED_FAILSAFE_ENABLED', 'Enable Advanced Failsafe', 0, "ADVANCED_FAILSAFE"),  # NOQA: 501
    Feature('Rover', 'ROVER_AUTO_ARM_ONCE', 'AP_ROVER_AUTO_ARM_ONCE_ENABLED', 'Make Auto-Arm-Once functionality available', 0, None),  # NOQA: 501

    Feature('Mission', 'MISSION_NAV_PAYLOAD_PLACE', 'AP_MISSION_NAV_PAYLOAD_PLACE_ENABLED', 'Enable NAV_PAYLOAD_PLACE', 0, None),  # noqa
    Feature('Copter', 'AC_PAYLOAD_PLACE_ENABLED', 'AC_PAYLOAD_PLACE_ENABLED', 'Enable Copter Payload Place', 0, 'MISSION_NAV_PAYLOAD_PLACE'),  # noqa

    Feature('Compass', 'AK09916', 'AP_COMPASS_AK09916_ENABLED', 'Enable AK0991x compasses', 1, None),
    Feature('Compass', 'AK8963', 'AP_COMPASS_AK8963_ENABLED', 'Enable AK8963 compasses', 1, None),
    Feature('Compass', 'BMM150', 'AP_COMPASS_BMM150_ENABLED', 'Enable BMM150 compasses', 1, None),
    Feature('Compass', 'BMM350', 'AP_COMPASS_BMM350_ENABLED', 'Enable BMM350 compasses', 1, None),
    Feature('Compass', 'EXTERNALAHRS_COMPASS', 'AP_COMPASS_EXTERNALAHRS_ENABLED', 'Enable ExternalAHRS compasses', 0, "AHRS_EXT"),  # noqa
    Feature('Compass', 'HMC5843', 'AP_COMPASS_HMC5843_ENABLED', 'Enable HMC5843 compasses', 1, None),
    Feature('Compass', 'ICM20948', 'AP_COMPASS_ICM20948_ENABLED', 'Enable AK0991x on ICM20948 compasses', 1, "AK09916"),
    Feature('Compass', 'IST8308', 'AP_COMPASS_IST8308_ENABLED', 'Enable IST8308 compasses', 1, None),
    Feature('Compass', 'IIS2MDC', 'AP_COMPASS_IIS2MDC_ENABLED', 'Enable IIS2MDC compasses', 0, None),
    Feature('Compass', 'IST8310', 'AP_COMPASS_IST8310_ENABLED', 'Enable IST8310 compasses', 1, None),
    Feature('Compass', 'LIS3MDL', 'AP_COMPASS_LIS3MDL_ENABLED', 'Enable LIS3MDL compasses', 1, None),
    Feature('Compass', 'LIS2MDL', 'AP_COMPASS_LIS2MDL_ENABLED', 'Enable LIS2MDL compasses', 1, None),
    Feature('Compass', 'LSM303D', 'AP_COMPASS_LSM303D_ENABLED', 'Enable LSM303D compasses', 1, None),
    Feature('Compass', 'LSM9DS1', 'AP_COMPASS_LSM9DS1_ENABLED', 'Enable LSM9DS1 compasses', 1, None),
    Feature('Compass', 'MAG3110', 'AP_COMPASS_MAG3110_ENABLED', 'Enable MAG3110 compasses', 1, None),
    Feature('Compass', 'MMC3416', 'AP_COMPASS_MMC3416_ENABLED', 'Enable MMC3416 compasses', 1, None),
    Feature('Compass', 'MMC5XX3', 'AP_COMPASS_MMC5XX3_ENABLED', 'Enable MMC5XX3 compasses', 1, None),
    Feature('Compass', 'QMC5883L', 'AP_COMPASS_QMC5883L_ENABLED', 'Enable QMC5883L compasses', 1, None),
    Feature('Compass', 'QMC5883P', 'AP_COMPASS_QMC5883P_ENABLED', 'Enable QMC5883P compasses', 1, None),
    Feature('Compass', 'RM3100', 'AP_COMPASS_RM3100_ENABLED', 'Enable RM3100 compasses', 1, None),
    Feature('Compass', 'DRONECAN_COMPASS', 'AP_COMPASS_DRONECAN_ENABLED', 'Enable DroneCAN compasses', 0, "DroneCAN"),
    Feature('Compass', 'DRONECAN_COMPASS_HIRES', 'AP_COMPASS_DRONECAN_HIRES_ENABLED', 'Enable DroneCAN HiRes compasses for survey logging', 0, "DroneCAN,DRONECAN_COMPASS"), # noqa
    Feature('Compass', 'FixedYawCal', 'AP_COMPASS_CALIBRATION_FIXED_YAW_ENABLED', 'Enable Fixed-Yaw Compass Calibration', 1, None),  # noqa
    Feature('Compass', 'CompassLearn', 'COMPASS_LEARN_ENABLED', 'Enable In-Flight compass learning', 1, "FixedYawCal"),

    Feature('Gimbal', 'MOUNT', 'HAL_MOUNT_ENABLED', 'Enable Camera Mounts', 0, None),
    Feature('Gimbal', 'ALEXMOS', 'HAL_MOUNT_ALEXMOS_ENABLED', 'Enable Alexmos gimbal', 0, "MOUNT"),
    Feature('Gimbal', 'CADDX', 'HAL_MOUNT_CADDX_ENABLED', 'Enable CADDX gimbal', 0, "MOUNT"),
    Feature('Gimbal', 'MAVLINK', 'HAL_MOUNT_MAVLINK_ENABLED', 'Enable MAVLink gimbal', 0, "MOUNT"),
    Feature('Gimbal', 'SERVO', 'HAL_MOUNT_SERVO_ENABLED', 'Enable Servo gimbal', 0, "MOUNT"),
    Feature('Gimbal', 'SIYI', 'HAL_MOUNT_SIYI_ENABLED', 'Enable Siyi gimbal', 0, "MOUNT"),
    Feature('Gimbal', 'SOLOGIMBAL', 'HAL_SOLO_GIMBAL_ENABLED', 'Enable Solo gimbal', 0, "MOUNT"),
    Feature('Gimbal', 'STORM32_MAVLINK', 'HAL_MOUNT_STORM32MAVLINK_ENABLED', 'Enable SToRM32 MAVLink gimbal', 0, "MOUNT"),
    Feature('Gimbal', 'STORM32_SERIAL', 'HAL_MOUNT_STORM32SERIAL_ENABLED', 'Enable SToRM32 Serial gimbal', 0, "MOUNT"),
    Feature('Gimbal', 'TOPOTEK', 'HAL_MOUNT_TOPOTEK_ENABLED', 'Enable Topotek gimbal', 0, "MOUNT"),
    Feature('Gimbal', 'XACTI', 'HAL_MOUNT_XACTI_ENABLED', 'Enable Xacti gimbal', 0, "MOUNT,DroneCAN"),
    Feature('Gimbal', 'XFROBOT', 'HAL_MOUNT_XFROBOT_ENABLED', 'Enable XFRobot gimbal', 0, "MOUNT"),
    Feature('Gimbal', 'VIEWPRO', 'HAL_MOUNT_VIEWPRO_ENABLED', 'Enable Viewpro gimbal', 0, "MOUNT"),

    Feature('VTOL Frame', 'TRI', 'AP_MOTORS_TRI_ENABLED', 'TriCopters', 0, None),
    Feature('VTOL Frame', 'QUAD', 'AP_MOTORS_FRAME_QUAD_ENABLED', 'QUAD', 1, None),
    Feature('VTOL Frame', 'HEXA', 'AP_MOTORS_FRAME_HEXA_ENABLED', 'HEXA', 0, None),
    Feature('VTOL Frame', 'OCTA', 'AP_MOTORS_FRAME_OCTA_ENABLED', 'OCTA', 0, None),
    Feature('VTOL Frame', 'DECA', 'AP_MOTORS_FRAME_DECA_ENABLED', 'DECA', 0, None),
    Feature('VTOL Frame', 'DODECAHEXA', 'AP_MOTORS_FRAME_DODECAHEXA_ENABLED', 'DODECAHEXA', 0, None),
    Feature('VTOL Frame', 'Y6', 'AP_MOTORS_FRAME_Y6_ENABLED', 'Y6', 0, None),
    Feature('VTOL Frame', 'OCTAQUAD', 'AP_MOTORS_FRAME_OCTAQUAD_ENABLED', 'OCTAQUAD', 0, None),

    Feature('Payload', 'GRIPPER', 'AP_GRIPPER_ENABLED', 'Enable Gripper', 0, None),
    Feature('Payload', 'SPRAYER', 'HAL_SPRAYER_ENABLED', 'Enable Sprayer', 0, None),
    Feature('Payload', 'LANDING_GEAR', 'AP_LANDINGGEAR_ENABLED', 'Enable Landing Gear', 0, None),
    Feature('Payload', 'WINCH', 'AP_WINCH_ENABLED', 'Enable Winch', 0, None),
    Feature('Payload', 'WINCH_DAIWA', 'AP_WINCH_DAIWA_ENABLED', 'Enable DAIWA Winch', 0, 'WINCH'),
    Feature('Payload', 'WINCH_PWM', 'AP_WINCH_PWM_ENABLED', 'Enable PWM Winch', 0, 'WINCH'),

    Feature('Payload', 'RELAY', 'AP_RELAY_ENABLED', 'Enable Relays', 0, None),
    Feature('Payload', 'SERVORELAY_EVENTS', 'AP_SERVORELAYEVENTS_ENABLED', 'Enable Servo/Relay Event', 0, None),

    Feature('Plane', 'ADVANCED_FAILSAFE', 'AP_ADVANCEDFAILSAFE_ENABLED', 'Enable Advanced Failsafe', 0, None),
    Feature('Plane', 'QUADPLANE', 'HAL_QUADPLANE_ENABLED', 'Enable QuadPlane', 0, None),
    Feature('Plane', 'SOARING', 'HAL_SOARING_ENABLED', 'Enable Soaring', 0, None),
    Feature('Plane', 'DEEPSTALL', 'HAL_LANDING_DEEPSTALL_ENABLED', 'Enable Deepstall landing', 0, None),
    Feature('Plane', 'QAUTOTUNE', 'QAUTOTUNE_ENABLED', 'Enable QuadPlane AUTOTUNE', 0, "QUADPLANE"),
    Feature('Plane', 'PLANE_BLACKBOX', 'AP_PLANE_BLACKBOX_LOGGING', 'Enable Blackbox logging', 0, None),
    Feature('Plane', 'AP_TX_TUNING', 'AP_TUNING_ENABLED', 'Enable TX-based tuning parameter adjustments', 0, None),
    Feature('Plane', 'PLANE_GUIDED_SLEW', 'AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED', 'Enable Offboard-guided slew commands', 0, None),  # noqa:401
    Feature('Plane', 'PLANE_GLIDER_PULLUP', 'AP_PLANE_GLIDER_PULLUP_ENABLED', 'Enable Glider pullup support', 0, None),
    Feature('Plane', 'QUICKTUNE', 'AP_QUICKTUNE_ENABLED', 'Enable VTOL quicktune', 0, None),
    Feature('Plane', 'AUTOLAND_MODE', 'MODE_AUTOLAND_ENABLED', 'Enable Fixed Wing Autolanding mode', 0, None),
    Feature('Plane', 'PLANE_SYSTEMID', 'AP_PLANE_SYSTEMID_ENABLED', 'Enable systemID support', 0, 'QUADPLANE,Logging'),  # NOQA:E501

    Feature('RC', 'RC_Protocol', 'AP_RCPROTOCOL_ENABLED', "Enable Serial RC Protocols", 0, None),   # NOQA: E501
    Feature('RC', 'RC_CRSF', 'AP_RCPROTOCOL_CRSF_ENABLED', "Enable CRSF", 0, "RC_Protocol"),   # NOQA: E501
    Feature('RC', 'RC_IBUS', 'AP_RCPROTOCOL_IBUS_ENABLED', "Enable IBus", 0, "RC_Protocol"),   # NOQA: E501
    Feature('RC', 'RC_SBUS', 'AP_RCPROTOCOL_SBUS_ENABLED', "Enable SBUS", 0, "RC_Protocol"),   # NOQA: E501
    Feature('RC', 'RC_PPMSUM', 'AP_RCPROTOCOL_PPMSUM_ENABLED', "Enable PPMSum", 0, "RC_Protocol"),   # NOQA: E501
    Feature('RC', 'RC_SRXL', 'AP_RCPROTOCOL_SRXL_ENABLED', "Enable SRXL", 0, "RC_Protocol"),   # NOQA: E501
    Feature('RC', 'RC_SRXL2', 'AP_RCPROTOCOL_SRXL2_ENABLED', "Enable SRXL2", 0, "RC_Protocol"),   # NOQA: E501
    Feature('RC', 'RC_ST24', 'AP_RCPROTOCOL_ST24_ENABLED', "Enable ST24", 0, "RC_Protocol"),   # NOQA: E501
    Feature('RC', 'RC_SUMD', 'AP_RCPROTOCOL_SUMD_ENABLED', "Enable SUMD", 0, "RC_Protocol"),   # NOQA: E501
    Feature('RC', 'RC_GHST', 'AP_RCPROTOCOL_GHST_ENABLED', "Enable Ghost", 0, "RC_Protocol"),   # NOQA: E501
    Feature('RC', 'RC_MAVLINK_RADIO', 'AP_RCPROTOCOL_MAVLINK_RADIO_ENABLED', "Enable MAVLink", 0, "RC_Protocol"),   # NOQA: E501
    Feature('RC', 'RSSI', 'AP_RSSI_ENABLED', 'RSSI', 0, None),

    Feature('RC', 'RC_TRANSMITTER_TUNING', 'AP_RC_TRANSMITTER_TUNING_ENABLED', "Enable knob-based transmitter tuning", 0, None),  # NOQA:E501

    Feature('Rangefinder', 'RANGEFINDER', 'AP_RANGEFINDER_ENABLED', "Enable Rangefinders", 0, None),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_ANALOG', 'AP_RANGEFINDER_ANALOG_ENABLED', "Enable Rangefinder - Analog", 0, "RANGEFINDER"),   # NOQA: E501
    # Feature('Rangefinder', 'RANGEFINDER_BBB_PRU', 'AP_RANGEFINDER_BBB_PRU_ENABLED', "Enable Rangefinder - BBB PRU", 0, "RANGEFINDER"),   # NOQA: E501
    # Feature('Rangefinder', 'RANGEFINDER_BEBOP', 'AP_RANGEFINDER_BEBOP_ENABLED', "Enable Rangefinder - Bebop", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_BENEWAKE_CAN', 'AP_RANGEFINDER_BENEWAKE_CAN_ENABLED', "Enable Rangefinder - Benewake (CAN)", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_BENEWAKE_TF02', 'AP_RANGEFINDER_BENEWAKE_TF02_ENABLED', "Enable Rangefinder - Benewake -TF02", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_BENEWAKE_TF03', 'AP_RANGEFINDER_BENEWAKE_TF03_ENABLED', "Enable Rangefinder - Benewake - TF03", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RFND_BENEWAKE_TFMINI', 'AP_RANGEFINDER_BENEWAKE_TFMINI_ENABLED', "Enable Rangefinder - Benewake - TFMini", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RFND_BENEWAKE_TFMINIPLUS', 'AP_RANGEFINDER_BENEWAKE_TFMINIPLUS_ENABLED', "Enable Rangefinder - Benewake - TFMiniPlus", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_BLPING', 'AP_RANGEFINDER_BLPING_ENABLED', "Enable Rangefinder - BLPing", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_GYUS42V2', 'AP_RANGEFINDER_GYUS42V2_ENABLED', "Enable Rangefinder - GYUS42V2", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_HC_SR04', 'AP_RANGEFINDER_HC_SR04_ENABLED', "Enable Rangefinder - HC_SR04", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_JRE_SERIAL', 'AP_RANGEFINDER_JRE_SERIAL_ENABLED', "Enable Rangefinder - JRE_SERIAL", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_LANBAO', 'AP_RANGEFINDER_LANBAO_ENABLED', "Enable Rangefinder - Lanbao", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_LEDDARONE', 'AP_RANGEFINDER_LEDDARONE_ENABLED', "Enable Rangefinder - LeddarOne", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_LEDDARVU8', 'AP_RANGEFINDER_LEDDARVU8_ENABLED', "Enable Rangefinder - LeddarVU8", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_LIGHTWARE_SERIAL', 'AP_RANGEFINDER_LIGHTWARE_SERIAL_ENABLED', "Enable Rangefinder - Lightware (serial)", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_LUA', 'AP_RANGEFINDER_LUA_ENABLED', "Enable Rangefinder - Lua Scripting", 0, "RANGEFINDER,SCRIPTING"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_LWI2C', 'AP_RANGEFINDER_LWI2C_ENABLED', "Enable Rangefinder - Lightware (i2c)", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_MAVLINK', 'AP_RANGEFINDER_MAVLINK_ENABLED', "Enable Rangefinder - MAVLink", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_MAXBOTIX_SERIAL', 'AP_RANGEFINDER_MAXBOTIX_SERIAL_ENABLED', "Enable Rangefinder - MaxBotix (serial)", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_MAXSONARI2CXL', 'AP_RANGEFINDER_MAXSONARI2CXL_ENABLED', "Enable Rangefinder - MaxSonarI2CXL", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_NMEA', 'AP_RANGEFINDER_NMEA_ENABLED', "Enable Rangefinder - NMEA", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_NOOPLOOP', 'AP_RANGEFINDER_NOOPLOOP_ENABLED', "Enable Rangefinder - Nooploop TOF P/F", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_NRA24_CAN', 'AP_RANGEFINDER_NRA24_CAN_ENABLED', "Enable Rangefinder - NRA24 CAN", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_PULSEDLIGHTLRF', 'AP_RANGEFINDER_PULSEDLIGHTLRF_ENABLED', "Enable Rangefinder - PulsedLightLRF", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_PWM', 'AP_RANGEFINDER_PWM_ENABLED', "Enable Rangefinder - PWM", 0, "RANGEFINDER"),   # NOQA: E501
    # Feature('Rangefinder', 'RANGEFINDER_SIM', 'AP_RANGEFINDER_SIM_ENABLED', "Enable Rangefinder - SIM", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_TOFSF_I2C', 'AP_RANGEFINDER_TOFSENSEF_I2C_ENABLED', "Enable Rangefinder - ToFSense-F I2C", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_TOFSP_CAN', 'AP_RANGEFINDER_TOFSENSEP_CAN_ENABLED', "Enable Rangefinder - ToFSense-P CAN", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_TRI2C', 'AP_RANGEFINDER_TRI2C_ENABLED', "Enable Rangefinder - TeraRangerI2C", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_TR_SERIAL', 'AP_RANGEFINDER_TERARANGER_SERIAL_ENABLED', "Enable Rangefinder - TeraRanger Serial", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_DRONECAN', 'AP_RANGEFINDER_DRONECAN_ENABLED', "Enable Rangefinder - DroneCAN", 0, "RANGEFINDER,DroneCAN"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_USD1_CAN', 'AP_RANGEFINDER_USD1_CAN_ENABLED', "Enable Rangefinder - USD1 (CAN)", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_USD1_SERIAL', 'AP_RANGEFINDER_USD1_SERIAL_ENABLED', "Enable Rangefinder - USD1 (SERIAL)", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_VL53L0X', 'AP_RANGEFINDER_VL53L0X_ENABLED', "Enable Rangefinder - VL53L0X", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_VL53L1X', 'AP_RANGEFINDER_VL53L1X_ENABLED', "Enable Rangefinder - VL53L1X", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_WASP', 'AP_RANGEFINDER_WASP_ENABLED', "Enable Rangefinder - Wasp", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_RDS02UF', 'AP_RANGEFINDER_RDS02UF_ENABLED', "Enable Rangefinder - RDS02UF", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_HEXSOONRADAR', 'AP_RANGEFINDER_HEXSOONRADAR_ENABLED', "Enable Rangefinder - Hexsoon Radar", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_LRD1', 'AP_RANGEFINDER_AINSTEIN_LR_D1_ENABLED', "Enable Rangefinder - Ainstein LRD1", 0, "RANGEFINDER"),   # NOQA: E501

    Feature('Sensors', 'OPTICALFLOW', 'AP_OPTICALFLOW_ENABLED', 'Enable Optical Flow', 0, None),
    Feature('Sensors', 'OPTICALFLOW_CXOF', 'AP_OPTICALFLOW_CXOF_ENABLED', 'Enable Optical flow CXOF Sensor', 0, "OPTICALFLOW"),
    Feature('Sensors', 'OPTICALFLOW_HEREFLOW', 'AP_OPTICALFLOW_HEREFLOW_ENABLED', 'Enable Optical flow HereFlow Sensor', 0, "OPTICALFLOW,DroneCAN"),   # NOQA: E501
    Feature('Sensors', 'OPTICALFLOW_MAV', 'AP_OPTICALFLOW_MAV_ENABLED', 'Enable Optical flow MAVLink Sensor', 0, "OPTICALFLOW"),   # NOQA: E501
    Feature('Sensors', 'OPTICALFLOW_ONBOARD', 'AP_OPTICALFLOW_ONBOARD_ENABLED', 'Enable Optical flow ONBOARD Sensor', 0, "OPTICALFLOW"),   # NOQA: E501
    Feature('Sensors', 'OPTICALFLOW_PX4FLOW', 'AP_OPTICALFLOW_PX4FLOW_ENABLED', 'Enable Optical flow PX4FLOW Sensor', 0, "OPTICALFLOW"),   # NOQA: E501
    Feature('Sensors', 'OPTICALFLOW_PIXART', 'AP_OPTICALFLOW_PIXART_ENABLED', 'Enable Optical flow PIXART Sensor', 0, "OPTICALFLOW"),   # NOQA: E501
    Feature('Sensors', 'OPTICALFLOW_UPFLOW', 'AP_OPTICALFLOW_UPFLOW_ENABLED', 'Enable Optical flow UPFLOW Sensor', 0, "OPTICALFLOW"),   # NOQA: E501

    Feature('Proximity', 'PROXIMITY', 'HAL_PROXIMITY_ENABLED', 'Enable Proximity', 0, None),
    Feature('Proximity', 'PROXIMITY_CYGBOT', 'AP_PROXIMITY_CYGBOT_ENABLED', 'Enable Cygbot D1 Proximity Sensors', 0, "PROXIMITY"),  # noqa
    Feature('Proximity', 'PROXIMITY_DRONECAN', 'AP_PROXIMITY_DRONECAN_ENABLED', 'Enable DroneCAN Proximity Sensors', 0, "PROXIMITY,DroneCAN"),  # noqa
    Feature('Proximity', 'PROXIMITY_LIGHTWARE_SF40C', 'AP_PROXIMITY_LIGHTWARE_SF40C_ENABLED', 'Enable LightWare SF40C Proximity Sensors', 0, "PROXIMITY"),  # noqa
    Feature('Proximity', 'PROXIMITY_LIGHTWARE_SF45B', 'AP_PROXIMITY_LIGHTWARE_SF45B_ENABLED', 'Enable LightWare SF45B Proximity Sensors', 0, "PROXIMITY"),  # noqa
    Feature('Proximity', 'PROXIMITY_MAV', 'AP_PROXIMITY_MAV_ENABLED', 'Enable MAVLink Proximity Sensors', 0, "PROXIMITY"),  # noqa
    Feature('Proximity', 'PROXIMITY_RANGEFINDER', 'AP_PROXIMITY_RANGEFINDER_ENABLED', 'Use RangeFinders as proximity sensors', 0, "PROXIMITY,RANGEFINDER"),  # noqa
    Feature('Proximity', 'PROXIMITY_RPLIDARA2', 'AP_PROXIMITY_RPLIDARA2_ENABLED', 'Enable RPLidarA2 Proximity Sensors', 0, "PROXIMITY"),  # noqa
    Feature('Proximity', 'PROXIMITY_TERRARANGERTOWER', 'AP_PROXIMITY_TERARANGERTOWER_ENABLED', 'Enable TerraRangerTower Proximity Sensors', 0, "PROXIMITY"),  # noqa
    Feature('Proximity', 'PROXIMITY_TERRARANGERTOWEREVO', 'AP_PROXIMITY_TERARANGERTOWEREVO_ENABLED', 'Enable TerraRangerTower Evo Proximity Sensors', 0, "PROXIMITY"),  # noqa
    Feature('Proximity', 'PROXIMITY_MR72_ENABLED', 'AP_PROXIMITY_MR72_ENABLED', 'Enable NanoRadar MR72 Proximity Sensors', 0, "PROXIMITY"),  # noqa
    Feature('Proximity', 'PROXIMITY_HEXSOONRADAR_ENABLED', 'AP_PROXIMITY_HEXSOONRADAR_ENABLED', 'Enable Hexsoon Radar Proximity Sensors', 0, "PROXIMITY"),  # noqa

    Feature('Baro', 'BMP085', 'AP_BARO_BMP085_ENABLED', 'Enable BMP085 Barometric Sensor', 1, None),
    Feature('Baro', 'BMP280', 'AP_BARO_BMP280_ENABLED', 'Enable BMP280 Barometric Sensor', 1, None),
    Feature('Baro', 'BMP388', 'AP_BARO_BMP388_ENABLED', 'Enable BMP388 Barometric Sensor', 1, None),
    Feature('Baro', 'BMP581', 'AP_BARO_BMP581_ENABLED', 'Enable BMP581 Barometric Sensor', 1, None),
    Feature('Baro', 'DPS280', 'AP_BARO_DPS280_ENABLED', 'Enable DPS280/DPS310 Barometric Sensor', 1, None),
    # Feature('Baro', 'DUMMY', 'AP_BARO_DUMMY_ENABLED', 'Enable DUMMY Barometric Sensor', 0, None),
    Feature('Baro', 'EXTERNALAHRS_BARO', 'AP_BARO_EXTERNALAHRS_ENABLED', 'Enable EXTERNALAHRS Barometric Sensor', 0, 'AHRS_EXT'),  # NOQA
    Feature('Baro', 'FBM320', 'AP_BARO_FBM320_ENABLED', 'Enable FBM320 Barometric Sensor', 1, None),
    # Feature('Baro', 'ICM20789', 'AP_BARO_ICM20789_ENABLED', 'Enable ICM20789 Barometric Sensor', 1, None),
    Feature('Baro', 'KELLERLD', 'AP_BARO_KELLERLD_ENABLED', 'Enable KELLERLD Barometric Sensor', 1, None),
    Feature('Baro', 'LPS2XH', 'AP_BARO_LPS2XH_ENABLED', 'Enable LPS2XH Barometric Sensor', 1, None),
    Feature('Baro', 'MS5607', 'AP_BARO_MS5607_ENABLED', 'Enable MS5607 Barometric Sensor', 1, None),
    Feature('Baro', 'MS5611', 'AP_BARO_MS5611_ENABLED', 'Enable MS5611 Barometric Sensor', 1, 'MS5607'),  # MS5611 has an option to be treated as 5607  # NOQA:E501
    Feature('Baro', 'MS5637', 'AP_BARO_MS5637_ENABLED', 'Enable MS5637 Barometric Sensor', 1, None),
    Feature('Baro', 'MS5837', 'AP_BARO_MS5837_ENABLED', 'Enable MS5837 Barometric Sensor', 1, None),
    Feature('Baro', 'MSP_BARO', 'AP_BARO_MSP_ENABLED', 'Enable MSP Barometric Sensor', 0, 'MSP'),
    Feature('Baro', 'SPL06', 'AP_BARO_SPL06_ENABLED', 'Enable SPL06 Barometric Sensor', 1, None),
    Feature('Baro', 'DRONECAN_BARO', 'AP_BARO_DRONECAN_ENABLED', 'Enable DroneCAN Barometric Sensor', 0, "DroneCAN"),
    # Feature('Baro', 'ICP101XX', 'AP_BARO_ICP101XX_ENABLED', 'Enable ICP101XX Barometric Sensor', 0, None),
    # Feature('Baro', 'ICP201XX', 'AP_BARO_ICP201XX_ENABLED', 'Enable ICP201XX Barometric Sensor', 0, None),
    Feature('Baro', 'BARO_TEMPCAL', 'AP_TEMPCALIBRATION_ENABLED', 'Enable Baro Temperature Calibration', 0, None),
    Feature('Baro', 'BARO_PROBEXT', 'AP_BARO_PROBE_EXTERNAL_I2C_BUSES', 'Enable Probing of External i2c buses', 0, None),
    Feature('Baro', 'BARO_THRUST_COMP', 'AP_BARO_THST_COMP_ENABLED', 'Enable thrust compensation on BARO1', 0, None),

    Feature('Sensors', 'RPM', 'AP_RPM_ENABLED', 'Enable RPM sensors', 0, None),
    Feature('Sensors', 'RPM_EFI', 'AP_RPM_EFI_ENABLED', 'Enable RPM EFI sensors', 0, 'RPM,EFI'),
    Feature('Sensors', 'RPM_ESC_TELEM', 'AP_RPM_ESC_TELEM_ENABLED', 'Enable RPM ESC Telemetry sensors', 0, 'RPM'),
    Feature('Sensors', 'RPM_HARMONIC_NOTCH', 'AP_RPM_HARMONICNOTCH_ENABLED', 'Enable RPM Harmonic Notch sensors', 0, 'RPM,HarmonicNotches'),  # noqa
    Feature('Sensors', 'RPM_PIN', 'AP_RPM_PIN_ENABLED', 'Enable RPM Pin-based sensors', 0, 'RPM'),
    Feature('Sensors', 'RPM_GENERATOR', 'AP_RPM_GENERATOR_ENABLED', 'Enable Generator RPM sensors', 0, 'RPM,GENERATOR'),
    Feature('Sensors', 'RPM_DRONECAN', 'AP_RPM_DRONECAN_ENABLED', 'Enable DroneCAN-based RPM sensors', 0, 'RPM,GENERATOR,DroneCAN'),  # noqa

    Feature('Sensors', 'TEMP', 'AP_TEMPERATURE_SENSOR_ENABLED', 'Enable Temperature Sensors', 0, None),
    Feature('Sensors', 'TEMP_TSYS01', 'AP_TEMPERATURE_SENSOR_TSYS01_ENABLED', 'Enable Temp Sensor - TSYS01', 0, "TEMP"),
    Feature('Sensors', 'TEMP_MCP9600', 'AP_TEMPERATURE_SENSOR_MCP9600_ENABLED', 'Enable Temp Sensor - MCP9600', 0, "TEMP"),
    Feature('Sensors', 'TEMP_TSYS03', 'AP_TEMPERATURE_SENSOR_TSYS03_ENABLED', 'Enable Temp Sensor - TSYS03', 0, "TEMP"),
    Feature('Sensors', 'TEMP_MLX90614', 'AP_TEMPERATURE_SENSOR_MLX90614_ENABLED', 'Enable Temp Sensor - MLX90614', 0, "TEMP"),
    Feature('Sensors', 'TEMP_SHT3X', 'AP_TEMPERATURE_SENSOR_SHT3X_ENABLED', 'Enable Temp Sensor - SHT3x', 0, "TEMP"),

    Feature('Sensors', 'AIRSPEED', 'AP_AIRSPEED_ENABLED', 'Enable Airspeed Sensors', 1, None),    # Default to enabled to not annoy Plane users   # NOQA: E501
    Feature('Sensors', 'BEACON', 'AP_BEACON_ENABLED', 'Enable Beacon', 0, None),
    Feature('Sensors', 'GPS_MOVING_BASELINE', 'GPS_MOVING_BASELINE', 'Enable GPS Moving Baseline', 0, None),
    Feature('Sensors', 'IMU_ON_UART', 'AP_SERIALMANAGER_IMUOUT_ENABLED', 'Enable Send raw IMU data on a serial port', 0, None), # NOQA: E501

    Feature('IMU', 'TEMPCAL', 'HAL_INS_TEMPERATURE_CAL_ENABLE', 'Enable IMU Temperature calibration', 0, None),
    Feature('IMU', 'HarmonicNotches', 'AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED', 'Enable InertialSensor harmonic notch filters', 0, None),  # noqa
    Feature('IMU', 'BatchSampler', 'AP_INERTIALSENSOR_BATCHSAMPLER_ENABLED', 'Enable Batch sampler', 0, None),  # noqa

    Feature('Other', 'RateLoopThread', 'AP_INERTIALSENSOR_FAST_SAMPLE_WINDOW_ENABLED', 'Enable Rate Loop Thread', 0, 'HarmonicNotches'),  # noqa
    Feature('Other', 'GyroFFT', 'HAL_GYROFFT_ENABLED', 'Enable In-Flight gyro FFT calculations', 0, None),
    Feature('Other', 'NMEA_OUTPUT', 'HAL_NMEA_OUTPUT_ENABLED', 'Enable NMEA output', 0, None),
    Feature('Other', 'SDCARD_FORMATTING', 'AP_FILESYSTEM_FORMAT_ENABLED', 'Enable Formatting of microSD cards', 0, None),
    Feature('Other', 'BOOTLOADER_FLASHING', 'AP_BOOTLOADER_FLASHING_ENABLED', 'Enable Bootloader flashing', 0, "FILESYSTEM_ROMFS"),  # noqa
    Feature('Other', 'SERIALDEVICE_REGISTER', 'AP_SERIALMANAGER_REGISTER_ENABLED', 'Enable Serial device registration', 0, None), # noqa

    Feature('Scripting', 'SCRIPTING', 'AP_SCRIPTING_ENABLED', 'Enable Lua Scripting', 0, None),
    Feature('Scripting', 'SCRIPTING_SERIALDEVICE', 'AP_SCRIPTING_SERIALDEVICE_ENABLED', 'Enable Lua serial device simulation', 0, "SCRIPTING,SERIALDEVICE_REGISTER"), # noqa
    Feature('Scripting', 'SCRIPTING_BINDING_MOTORS', 'AP_SCRIPTING_BINDING_MOTORS_ENABLED', 'Enable bindings for AP_Motors', 0, "SCRIPTING"), # noqa
    Feature('Scripting', 'SCRIPTING_BINDING_VEHICLE', 'AP_SCRIPTING_BINDING_VEHICLE_ENABLED', 'Enable bindings for AP_Vehicle', 0, "SCRIPTING"), # noqa

    Feature('Other', 'SLCAN', 'AP_CAN_SLCAN_ENABLED', 'Enable SLCAN serial protocol', 0, None),
    Feature('Other', 'SDCARD_MISSION', 'AP_SDCARD_STORAGE_ENABLED', 'Enable Storing mission on microSD cards', 0, None),
    Feature('Other', 'COMPASS_CAL', 'COMPASS_CAL_ENABLED', 'Enable "Tumble" compass calibration', 0, None),
    Feature('Other', 'DRONECAN_SERIAL', 'AP_DRONECAN_SERIAL_ENABLED', 'Enable DroneCAN virtual serial ports', 0, "DroneCAN,SERIALDEVICE_REGISTER"),  # NOQA: E501
    Feature('Other', 'Buttons', 'HAL_BUTTON_ENABLED', 'Enable Buttons', 0, None),
    Feature('Other', 'Logging', 'HAL_LOGGING_ENABLED', 'Enable Logging', 0, None),
    Feature('Other', 'CUSTOM_ROTATIONS', 'AP_CUSTOMROTATIONS_ENABLED', 'Enable Custom  sensor rotations', 0, None),
    Feature('Other', 'PID_FILTERING', 'AP_FILTER_ENABLED', 'Enable PID filtering', 0, None),
    Feature('Other', 'POLYFENCE_CIRCLE_INT_SUPPORT', 'AC_POLYFENCE_CIRCLE_INT_SUPPORT_ENABLED', 'Fence circle compatability', 0, None),  # NOQA:E501
    Feature('Other', 'AP_AVOIDANCE', 'AP_ADSB_AVOIDANCE_ENABLED', 'Enable "ADSB" Avoidance', 0, 'ADSB'),
    Feature('Other', 'CPU_IDLE_STATS', 'AP_CPU_IDLE_STATS_ENABLED', 'Enable CPU idle stats', 0, None), # NOQA:E501

    # MAVLink section for mavlink features and/or message handling,
    # rather than for e.g. mavlink-based sensor drivers
    Feature('MAVLink', 'HIGHLAT2', 'HAL_HIGH_LATENCY2_ENABLED', 'Enable HighLatency2 Support', 0, None),
    Feature('MAVLink', 'FENCEPOINT_PROTOCOL', 'AC_POLYFENCE_FENCE_POINT_PROTOCOL_SUPPORT', 'Enable Old MAVLink fence points protocol', 0, "FENCE"),  # noqa
    Feature('MAVLink', 'RALLYPOINT_PROTOCOL', 'AP_MAVLINK_RALLY_POINT_PROTOCOL_ENABLED', 'Enable Old MAVLink rally points protocol', 0, "RALLY"),  # noqa
    Feature('MAVLink', 'MAVLINK_VERSION_REQUEST', 'AP_MAVLINK_AUTOPILOT_VERSION_REQUEST_ENABLED', 'Enable Old AUTOPILOT_VERSION_REQUEST mesage', 0, None),  # noqa
    Feature('MAVLink', 'REQUEST_AUTOPILOT_CAPA', 'AP_MAVLINK_MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES_ENABLED', 'Enable Old REQUEST_AUTOPILOT_CAPABILITIES command', 0, None),  # noqa
    Feature('MAVLink', 'MAV_MSG_RELAY_STATUS', 'AP_MAVLINK_MSG_RELAY_STATUS_ENABLED', 'Enable Send RELAY_STATUS message', 0, 'RELAY'),  # noqa
    Feature('MAVLink', 'MAV_DEVICE_OP', 'AP_MAVLINK_MSG_DEVICE_OP_ENABLED', 'Enable DeviceOp MAVLink messages', 0, None),  # noqa
    Feature('MAVLink', 'MAV_SERVO_RELAY', 'AP_MAVLINK_SERVO_RELAY_ENABLED', 'Enable ServoRelay MAVLink messages', 0, 'SERVORELAY_EVENTS'),  # noqa
    Feature('MAVLink', 'MAV_MSG_SERIAL_CONTROL', 'AP_MAVLINK_MSG_SERIAL_CONTROL_ENABLED', 'Enable Serial Control MAVLink messages', 0, None),  # noqa
    Feature('MAVLink', 'MAVLINK_MSG_MISSION_REQUEST', 'AP_MAVLINK_MSG_MISSION_REQUEST_ENABLED', 'Enable MISSION_REQUEST MAVLink messages', 0, None),  # noqa
    Feature('MAVLink', 'MAVLINK_MSG_RC_CHANNELS_RAW', 'AP_MAVLINK_MSG_RC_CHANNELS_RAW_ENABLED', 'Enable RC_CHANNELS_RAW MAVLink messages', 0, None),  # noqa
    Feature('MAVLink', 'AP_MAVLINK_FTP_ENABLED', 'AP_MAVLINK_FTP_ENABLED', 'Enable MAVLink FTP protocol', 0, None),  # noqa
    Feature('MAVLink', 'MAV_CMD_SET_HAGL', 'AP_MAVLINK_MAV_CMD_SET_HAGL_ENABLED', 'Enable MAVLink HAGL command', 0, None),  # noqa
    Feature('MAVLink', 'VIDEO_STREAM_INFORMATION', 'AP_MAVLINK_MSG_VIDEO_STREAM_INFORMATION_ENABLED', 'Enable MAVLink VIDEO_STREAM_INFORMATION message', 0, "Camera"), # noqa
    Feature('Other', 'AP_FOLLOW', 'AP_FOLLOW_ENABLED', 'Enable Follow library', 0, None),
    Feature('MAVLink', 'MAVLINK_MSG_FLIGHT_INFORMATION', 'AP_MAVLINK_MSG_FLIGHT_INFORMATION_ENABLED', 'Enable FLIGHT_INFORMATION MAVLink message', 0, None),  # noqa
    Feature('MAVLink', 'RANGEFINDER_SENDING', 'AP_MAVLINK_MSG_RANGEFINDER_SENDING_ENABLED', 'Enable sending of RANGEFINDER mavlink message', 0, "RANGEFINDER"),  # noqa
    Feature('MAVLink', 'MAVLINK_SIGNING', 'AP_MAVLINK_SIGNING_ENABLED', 'Enable MAVLink2 packet signing and validation', 0, "RANGEFINDER"),  # noqa

    Feature('Developer', 'KILL_IMU', 'AP_INERTIALSENSOR_KILL_IMU_ENABLED', 'Allow IMUs to be disabled at runtime', 0, None),
    Feature('Developer', 'CRASHCATCHER', 'AP_CRASHDUMP_ENABLED', 'Enable CrashCatcher', 0, None),

    Feature('GPS Drivers', 'UBLOX', 'AP_GPS_UBLOX_ENABLED', 'Enable U-blox GPS', 1, None),
    Feature('GPS Drivers', 'SBP2', 'AP_GPS_SBP2_ENABLED', 'Enable SBP2 GPS', 0, 'SBP'),
    Feature('GPS Drivers', 'SBP', 'AP_GPS_SBP_ENABLED', 'Enable SBP GPS', 0, None),
    Feature('GPS Drivers', 'ERB', 'AP_GPS_ERB_ENABLED', 'Enable ERB GPS', 0, None),
    Feature('GPS Drivers', 'GSOF', 'AP_GPS_GSOF_ENABLED', 'Enable GSOF GPS', 0, None),
    Feature('GPS Drivers', 'NMEA_GPS', 'AP_GPS_NMEA_ENABLED', 'Enable NMEA GPS', 0, None),
    Feature('GPS Drivers', 'NMEA_UNICORE', 'AP_GPS_NMEA_UNICORE_ENABLED', 'Enable NMEA Unicore GPS', 0, "NMEA_GPS"),
    Feature('GPS Drivers', 'MAV', 'AP_GPS_MAV_ENABLED', 'Enable MAVLink GPS', 0, None),
    Feature('GPS Drivers', 'NOVA', 'AP_GPS_NOVA_ENABLED', 'Enable NOVA GPS', 0, None),
    Feature('GPS Drivers', 'SBF', 'AP_GPS_SBF_ENABLED', 'Enable SBF GPS', 0, None),
    Feature('GPS Drivers', 'SIRF', 'AP_GPS_SIRF_ENABLED', 'Enable SiRF GPS', 0, None),
    Feature('GPS Drivers', 'DroneCAN_GPS_Out', 'AP_DRONECAN_SEND_GPS', 'Enable Sending GPS data from Autopilot', 0, "DroneCAN"),  # noqa:401
    Feature('GPS Drivers', 'GPS_Blending', 'AP_GPS_BLENDED_ENABLED', 'Enable GPS Blending', 0, None),


    Feature('Airspeed Drivers', 'Analog', 'AP_AIRSPEED_ANALOG_ENABLED', 'Enable Analog Airspeed', 0, 'AIRSPEED'),
    Feature('Airspeed Drivers', 'ASP5033', 'AP_AIRSPEED_ASP5033_ENABLED', 'Enable ASP5033 AIRSPEED', 0, 'AIRSPEED'),  # NOQA: E501
    Feature('Airspeed Drivers', 'DLVR', 'AP_AIRSPEED_DLVR_ENABLED', 'Enable DLVR AIRSPEED', 0, 'AIRSPEED'),
    Feature('Airspeed Drivers', 'MS4525', 'AP_AIRSPEED_MS4525_ENABLED', 'Enable MS4525 AIRSPEED', 0, 'AIRSPEED'),
    Feature('Airspeed Drivers', 'MS5525', 'AP_AIRSPEED_MS5525_ENABLED', 'Enable MS5525 AIRSPEED', 0, 'AIRSPEED'),
    Feature('Airspeed Drivers', 'MSP_AIRSPEED', 'AP_AIRSPEED_MSP_ENABLED', 'Enable MSP AIRSPEED', 0, 'AIRSPEED,MSP,OSD'),
    Feature('Airspeed Drivers', 'NMEA_AIRSPEED', 'AP_AIRSPEED_NMEA_ENABLED', 'Enable NMEA AIRSPEED', 0, 'AIRSPEED'),
    Feature('Airspeed Drivers', 'SDP3X', 'AP_AIRSPEED_SDP3X_ENABLED', 'Enable SDP3X AIRSPEED', 0, 'AIRSPEED'),
    Feature('Airspeed Drivers', 'DRONECAN_ASPD', 'AP_AIRSPEED_DRONECAN_ENABLED', 'Enable DroneCAN AIRSPEED', 0, 'AIRSPEED,DroneCAN'),   # NOQA: E501
    Feature('Airspeed Drivers', 'AUAV_AIRSPEED', 'AP_AIRSPEED_AUAV_ENABLED', 'ENABLE AUAV AIRSPEED', 0, 'AIRSPEED'),

    Feature('Actuators', 'ServoTelem', 'AP_SERVO_TELEM_ENABLED', 'Enable servo telemetry library', 0, None),
    Feature('Actuators', 'Volz', 'AP_VOLZ_ENABLED', 'Enable Volz Protocol', 0, None),
    Feature('Actuators', 'Volz_DroneCAN', 'AP_DRONECAN_VOLZ_FEEDBACK_ENABLED', 'Enable Volz DroneCAN Feedback', 0, "DroneCAN,Volz,ServoTelem"),  # noqa: E501
    Feature('Actuators', 'RobotisServo', 'AP_ROBOTISSERVO_ENABLED', 'Enable RobotisServo protocol', 0, None),
    Feature('Actuators', 'SBUS Output', 'AP_SBUSOUTPUT_ENABLED', 'Enable SBUS output on serial ports', 0, None),
    Feature('Actuators', 'FETTecOneWire', 'AP_FETTEC_ONEWIRE_ENABLED', 'Enable FETTec OneWire ESCs', 0, None),
    Feature('Actuators', 'KDECAN', 'AP_KDECAN_ENABLED', 'KDE Direct KDECAN ESC', 0, None),
    Feature('Actuators', 'HimarkServo', 'AP_DRONECAN_HIMARK_SERVO_SUPPORT', 'Enable Himark DroneCAN servos', 0, "DroneCAN"),
    Feature('Actuators', 'HobbywingESC', 'AP_DRONECAN_HOBBYWING_ESC_SUPPORT', 'Enable Hobbywing DroneCAN ESCs', 0, "DroneCAN"),

    Feature('Precision Landing', 'PrecLand', 'AC_PRECLAND_ENABLED', 'Enable Precision landing support', 0, None),
    Feature('Precision Landing', 'PrecLand - Companion', 'AC_PRECLAND_MAVLINK_ENABLED', 'Enable MAVLink precision landing support', 0, "PrecLand"),  # noqa
    Feature('Precision Landing', 'PrecLand - IRLock', 'AC_PRECLAND_IRLOCK_ENABLED', 'Enable IRLock precision landing support', 0, "PrecLand"),  # noqa

    #    Feature('Filesystem', 'FILESYSTEM_ESP32_ENABLED', 'AP_FILESYSTEM_ESP32_ENABLED', 'Enable ESP32 Filesystem', 0, None),
    # Feature('Filesystem', 'FILESYSTEM_FATFS', 'AP_FILESYSTEM_FATFS_ENABLED', 'Enable FATFS Filesystem', 0, None),
    Feature('Filesystem', 'FILESYSTEM_MISSION', 'AP_FILESYSTEM_MISSION_ENABLED', 'Enable @MISSION/ filesystem', 0, None),
    Feature('Filesystem', 'FILESYSTEM_PARAM', 'AP_FILESYSTEM_PARAM_ENABLED', 'Enable @PARAM/ filesystem', 0, None),
    #    Feature('Filesystem', 'FILESYSTEM_POSIX', 'AP_FILESYSTEM_POSIX_ENABLED', 'Enable POSIX filesystem', 0, None),
    Feature('Filesystem', 'FILESYSTEM_ROMFS', 'AP_FILESYSTEM_ROMFS_ENABLED', 'Enable @ROMFS/ filesystem', 0, None),
    Feature('Filesystem', 'FILESYSTEM_SYS', 'AP_FILESYSTEM_SYS_ENABLED', 'Enable @SYS/ filesystem', 0, None),
    Feature('Filesystem', 'APJ_TOOL_PARAMETERS', 'FORCE_APJ_DEFAULT_PARAMETERS', 'Enable apj_tool parameter area', 0, None),

    Feature('Networking', 'Networking', 'AP_NETWORKING_ENABLED', 'Enable networking', 0, None),
    Feature('Networking', 'PPP', 'AP_NETWORKING_BACKEND_PPP', 'Enable PPP networking', 0, "Networking"),
    Feature('Networking', 'CAN_MCAST', 'AP_NETWORKING_CAN_MCAST_ENABLED', 'Enable CAN multicast bridge', 0, "Networking,DroneCAN"), # noqa
    Feature('Networking', 'NetworkCapture', 'AP_NETWORKING_CAPTURE_ENABLED', 'Enable network capture', 0, "Networking"),

    Feature('CAN', 'DroneCAN', 'HAL_ENABLE_DRONECAN_DRIVERS', 'Enable DroneCAN support', 0, None),
    Feature('CAN', 'CAN Logging', 'AP_CAN_LOGGING_ENABLED', 'Enable CAN logging support', 0, 'Logging'),

    Feature('DDS', 'DDS', 'AP_DDS_ENABLED', 'Enable MicroXRCE DDS support for ROS 2', 0, None),

]

BUILD_OPTIONS.sort(key=lambda x: (x.category + x.label))

# sanity check the list to ensure names don't get too long.  These are
# used in various displays, so a good English "name" for the feature
# makes sense:
sanity_check_failed = False
for x in BUILD_OPTIONS:
    if len(x.label) > 30:
        sanity_check_failed = True
        print(f"{x.label} is too long")
        sanity_check_failed = True

if sanity_check_failed:
    raise ValueError("Bad labels in Feature list")
