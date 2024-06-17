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


# list of build options to offer NOTE: the dependencies must be
# written as a single string with commas and no spaces,
# eg. 'dependency1,dependency2'
BUILD_OPTIONS = [
    Feature('AHRS', 'EKF3', 'HAL_NAVEKF3_AVAILABLE', 'Enable EKF3', 1, None),
    Feature('AHRS', 'EKF2', 'HAL_NAVEKF2_AVAILABLE', 'Enable EKF2', 0, None),
    Feature('AHRS', 'AHRS_EXT', 'HAL_EXTERNAL_AHRS_ENABLED', 'Enable External AHRS', 0, None),
    Feature('AHRS', 'MicroStrain5', 'AP_EXTERNAL_AHRS_MICROSTRAIN5_ENABLED', 'Enable MICROSTRAIN 5-series External AHRS', 0, "AHRS_EXT"),  # noqa: E501
    Feature('AHRS', 'MicroStrain7', 'AP_EXTERNAL_AHRS_MICROSTRAIN7_ENABLED', 'Enable MICROSTRAIN 7-series External AHRS', 0, "AHRS_EXT"),  # noqa: E501
    Feature('AHRS', 'AHRS_EXT_VECTORNAV', 'AP_EXTERNAL_AHRS_VECTORNAV_ENABLED', 'Enable VectorNav External AHRS', 0, "AHRS_EXT"),  # noqa
    Feature('AHRS', 'TEMPCAL', 'HAL_INS_TEMPERATURE_CAL_ENABLE', 'Enable IMU Temperature Calibration', 0, None),
    Feature('AHRS', 'VISUALODOM', 'HAL_VISUALODOM_ENABLED', 'Enable Visual Odometry', 0, 'EKF3_EXTNAV'),
    Feature('AHRS', 'EKF3_EXTNAV', 'EK3_FEATURE_EXTERNAL_NAV', 'Enable External Navigation for EKF3', 0, 'EKF3'),
    Feature('AHRS', 'EKF3_WINDEST', 'EK3_FEATURE_DRAG_FUSION', 'Enable Wind Estimation for EKF3', 0, 'EKF3'),
    Feature('AHRS', 'BARO_WIND_COMP', 'HAL_BARO_WIND_COMP_ENABLED', 'Enable Baro Wind Compensation', 0, None),

    Feature('Safety', 'PARACHUTE', 'HAL_PARACHUTE_ENABLED', 'Enable Parachute', 0, None),
    Feature('Safety', 'FENCE', 'AP_FENCE_ENABLED', 'Enable Geofence', 2, None),
    Feature('Safety', 'RALLY', 'HAL_RALLY_ENABLED', 'Enable Rally Points', 0, None),  # noqa
    Feature('Safety', 'AC_AVOID', 'AP_AVOIDANCE_ENABLED', 'Enable Avoidance', 0, 'FENCE'),
    Feature('Safety', 'AC_OAPATHPLANNER', 'AP_OAPATHPLANNER_ENABLED', 'Enable Object Avoidance Path Planner', 0, 'FENCE'),

    Feature('Battery', 'BATTERY_FUELFLOW', 'AP_BATTERY_FUELFLOW_ENABLED', 'Enable Fuel Flow BatteryMonitor', 0, None),
    Feature('Battery', 'BATTERY_FUELLEVEL_PWM', 'AP_BATTERY_FUELLEVEL_PWM_ENABLED', 'Enable Flow Level PWM BatteryMonitor', 0, None),  # noqa: E501
    Feature('Battery', 'BATTERY_FUELLEVEL_ANALOG', 'AP_BATTERY_FUELLEVEL_ANALOG_ENABLED', 'Enable Flow Level Analog BattryMonitor', 0, None),  # noqa: E501
    Feature('Battery', 'BATTERY_SMBUS', 'AP_BATTERY_SMBUS_ENABLED', 'Enable SMBUS BatteryMonitor', 0, None),
    Feature('Battery', 'BATTERY_INA2XX', 'AP_BATTERY_INA2XX_ENABLED', 'Enable INA2XX BatteryMonitor', 0, None),
    Feature('Battery', 'BATTERY_SYNTHETIC_CURRENT', 'AP_BATTERY_SYNTHETIC_CURRENT_ENABLED', 'Enable Synthetic Current Monitor', 0, None), # noqa: E501
    Feature('Battery', 'BATTERY_ESC_TELEM_OUT', 'AP_BATTERY_ESC_TELEM_OUTBOUND_ENABLED', 'Enable ability to put battery monitor data in ESC telem stream', 0, None), # noqa: E501
    Feature('Battery', 'BATTERY_WATT_MAX', 'AP_BATTERY_WATT_MAX_ENABLED', 'Enable param BATT_WATT_MAX', 0, None), # noqa: E501

    Feature('Ident', 'ADSB', 'HAL_ADSB_ENABLED', 'Enable ADSB', 0, None),
    Feature('Ident', 'ADSB_SAGETECH', 'HAL_ADSB_SAGETECH_ENABLED', 'Enable Sagetech ADSB', 0, 'ADSB'),
    Feature('Ident', 'ADSB_SAGETECH_MXS', 'HAL_ADSB_SAGETECH_MXS_ENABLED', 'Enable Sagetech MXS ADSB', 0, 'ADSB'),
    Feature('Ident', 'ADSB_UAVIONIX', 'HAL_ADSB_UAVIONIX_MAVLINK_ENABLED', 'Enable UAvionix ADSB', 0, 'ADSB'),
    Feature('Ident', 'ADSB_UAVIONX_UCP', 'HAL_ADSB_UCP_ENABLED', 'Enable uAvionix UCP ADSB', 0 , 'ADSB'),
    Feature('Ident', 'AIS', 'AP_AIS_ENABLED', 'Enable AIS', 0, None),
    Feature('Ident', 'OpenDroneID', 'AP_OPENDRONEID_ENABLED', 'Enable OpenDroneID (Remote ID)', 0, None),

    Feature('Telemetry', 'CRSF', 'HAL_CRSF_TELEM_ENABLED', 'Enable CRSF Telemetry', 0, 'FrSky SPort PassThrough,FrSky,FrSky SPort,RC_CRSF'),  # noqa
    Feature('Telemetry', 'CRSFText', 'HAL_CRSF_TELEM_TEXT_SELECTION_ENABLED', 'Enable CRSF Text Param Selection', 0, 'CRSF,OSD_PARAM,FrSky SPort PassThrough,FrSky,FrSky SPort'),  # NOQA: E501
    Feature('Telemetry', 'HOTT', 'HAL_HOTT_TELEM_ENABLED', 'Enable HOTT Telemetry', 0, None),
    Feature('Telemetry', 'SPEKTRUM', 'HAL_SPEKTRUM_TELEM_ENABLED', 'Enable Spektrum Telemetry', 0, None),
    Feature('Telemetry', 'LTM', 'AP_LTM_TELEM_ENABLED', 'Enable LTM Telemetry', 0, None),
    Feature('Telemetry', 'AUX_FUNCTION_STRINGS', 'AP_RC_CHANNEL_AUX_FUNCTION_STRINGS_ENABLED', 'Enable Auxilliary Function activation text', 0, None),  # noqa
    Feature('Telemetry', 'FrSky', 'AP_FRSKY_TELEM_ENABLED', 'Enable FrSky Telemetry', 0, None),
    Feature('Telemetry', 'FrSky D', 'AP_FRSKY_D_TELEM_ENABLED', 'Enable FrSkyD Telemetry', 0, 'FrSky'),
    Feature('Telemetry', 'FrSky SPort', 'AP_FRSKY_SPORT_TELEM_ENABLED', 'Enable FrSkySPort Telemetry', 0, 'FrSky'),  # noqa
    Feature('Telemetry', 'FrSky SPort PassThrough', 'AP_FRSKY_SPORT_PASSTHROUGH_ENABLED', 'Enable FrSkySPort PassThrough Telemetry', 0, 'FrSky SPort,FrSky'),  # noqa
    Feature('Telemetry', 'GHST', 'AP_GHST_TELEM_ENABLED', 'Enable Ghost Telemetry', 0, "RC_GHST"), # noqa

    Feature('Notify', 'PLAY_TUNE', 'AP_NOTIFY_MAVLINK_PLAY_TUNE_SUPPORT_ENABLED', 'Enable MAVLink Play Tune', 0, None),  # noqa
    Feature('Notify', 'TONEALARM', 'AP_NOTIFY_TONEALARM_ENABLED', 'Enable ToneAlarm on PWM', 0, None),  # noqa
    Feature('Notify', 'LED_CONTROL', 'AP_NOTIFY_MAVLINK_LED_CONTROL_SUPPORT_ENABLED', 'Enable MAVLink LED Control', 0, None),  # noqa
    Feature('Notify', 'NOTIFY_NCP5623', 'AP_NOTIFY_NCP5623_ENABLED', 'Enable NCP5623 LED', 0, None),  # noqa
    # Feature('Notify', 'NOTIFY_PCA9685', 'AP_NOTIFY_PCA9685_ENABLED', 'Enable PCA9685 LED', 0, None),  # noqa  linux-only
    Feature('Notify', 'NOTIFY_PROFILED', 'AP_NOTIFY_PROFILED_ENABLED', 'Enable ProfiLED', 0, None),  # noqa
    Feature('Notify', 'DISPLAY', 'HAL_DISPLAY_ENABLED', 'Enable I2C Displays', 0, None),
    Feature('Notify', 'NOTIFY_PROFILED_SPI', 'AP_NOTIFY_PROFILED_SPI_ENABLED', 'Enable ProfiLED (SPI)', 0, None),  # noqa
    Feature('Notify', 'NOTIFY_NEOPIXEL', 'AP_NOTIFY_NEOPIXEL_ENABLED', 'Enable NeoPixel', 0, None),  # noqa

    Feature('MSP', 'MSP', 'HAL_MSP_ENABLED', 'Enable MSP Telemetry and MSP OSD', 0, 'OSD'),
    Feature('MSP', 'MSP_SENSORS', 'HAL_MSP_SENSORS_ENABLED', 'Enable MSP Sensors', 0, 'MSP_GPS,MSP_BARO,MSP_COMPASS,MSP_AIRSPEED,MSP,MSP_OPTICALFLOW,MSP_RANGEFINDER,OSD'),   # NOQA: E501
    Feature('MSP', 'MSP_GPS', 'HAL_MSP_GPS_ENABLED', 'Enable MSP GPS', 0, 'MSP,OSD'),
    Feature('MSP', 'MSP_COMPASS', 'AP_COMPASS_MSP_ENABLED', 'Enable MSP Compass', 0, 'MSP,OSD'),
    Feature('MSP', 'MSP_OPTICALFLOW', 'HAL_MSP_OPTICALFLOW_ENABLED', 'Enable MSP OpticalFlow', 0, 'MSP,OSD,OPTICALFLOW'), # also OPTFLOW dep   # NOQA: E501
    Feature('MSP', 'MSP_RANGEFINDER', 'HAL_MSP_RANGEFINDER_ENABLED', 'Enable MSP Rangefinder', 0, 'MSP,OSD,RANGEFINDER'),
    Feature('MSP', 'MSP_DISPLAYPORT', 'HAL_WITH_MSP_DISPLAYPORT', 'Enable MSP DisplayPort OSD (aka CANVAS MODE)', 0, 'MSP,OSD'),   # NOQA: E501

    Feature('ICE', 'ICE Engine', 'AP_ICENGINE_ENABLED', 'Enable Internal Combustion Engine support', 0, 'RPM'),
    Feature('ICE', 'EFI', 'HAL_EFI_ENABLED', 'Enable EFI Monitoring', 0, None),
    Feature('ICE', 'EFI_MegaSquirt', 'AP_EFI_SERIAL_MS_ENABLED', 'Enable EFI MegaSquirt', 0, 'EFI'),
    Feature('ICE', 'EFI_Lutan', 'AP_EFI_SERIAL_LUTAN_ENABLED', 'Enable EFI Lutan', 0, 'EFI'),
    Feature('ICE', 'EFI_NMPWU', 'AP_EFI_NWPWU_ENABLED', 'Enable EFI NMPMU', 0, 'EFI'),
    Feature('ICE', 'EFI_CURRAWONGECU', 'AP_EFI_CURRAWONG_ECU_ENABLED', 'Enable EFI Currawong ECU', 0, 'EFI'),
    Feature('ICE', 'EFI_HIRTH', 'AP_EFI_SERIAL_HIRTH_ENABLED', 'Enable EFI Hirth ECU', 0, 'EFI'),
    Feature('ICE', 'EFI_DRONECAN', 'AP_EFI_DRONECAN_ENABLED', 'Enable EFI DroneCAN', 0, 'EFI,DroneCAN'),
    Feature('ICE', 'EFI_MAV', 'AP_EFI_MAV_ENABLED', 'Enable EFI MAV', 0, 'EFI'),

    Feature('Generator', 'GENERATOR', 'HAL_GENERATOR_ENABLED', 'Enable Generator', 0, None),
    Feature('Generator', 'GENERATOR_RICHENPOWER', 'AP_GENERATOR_RICHENPOWER_ENABLED', 'Enable Richenpower Generator', 0, "GENERATOR"),  # noqa
    Feature('Generator', 'GENERATOR_IE2400', 'AP_GENERATOR_IE_2400_ENABLED', 'Enable IntelligentEnergy 2400', 0, "GENERATOR"),  # noqa
    Feature('Generator', 'GENERATOR_IE650', 'AP_GENERATOR_IE_650_800_ENABLED', 'Enable IntelligentEnergy 650 and 800 support', 0, "GENERATOR"),  # noqa

    Feature('OSD', 'OSD', 'OSD_ENABLED', 'Enable OSD', 0, None),
    Feature('OSD', 'PLUSCODE', 'HAL_PLUSCODE_ENABLE', 'Enable PlusCode', 0, 'OSD'),
    Feature('OSD', 'OSD_PARAM', 'OSD_PARAM_ENABLED', 'Enable OSD param', 0, 'OSD'),
    Feature('OSD', 'OSD_SIDEBARS', 'HAL_OSD_SIDEBAR_ENABLE', 'Enable Scrolling Sidebars', 0, 'OSD'),
    Feature('OSD', 'OSD_EXTENDED_LINK_STATS', 'AP_OSD_LINK_STATS_EXTENSIONS_ENABLED', 'Enable OSD panels with extended link stats data', 0, "OSD,RC_CRSF,MSP"),  # noqa

    Feature('VTX', 'VIDEO_TX', 'AP_VIDEOTX_ENABLED', 'Enable VideoTX control', 0, None),
    Feature('VTX', 'SMARTAUDIO', 'AP_SMARTAUDIO_ENABLED', 'Enable SmartAudio VTX Contol', 0, "VIDEO_TX"),
    Feature('VTX', 'TRAMP', 'AP_TRAMP_ENABLED', 'Enable IRC Tramp VTX Control', 0, "VIDEO_TX"),

    Feature('ESC', 'PICCOLOCAN', 'HAL_PICCOLO_CAN_ENABLE', 'Enable PiccoloCAN', 0, None),
    Feature('ESC', 'TORQEEDO', 'HAL_TORQEEDO_ENABLED', 'Enable Torqeedo Motors', 0, None),

    Feature('AP_Periph', 'LONG_TEXT', 'HAL_PERIPH_SUPPORT_LONG_CAN_PRINTF', 'Enable extended length text strings', 0, None),

    Feature('Camera', 'Camera', 'AP_CAMERA_ENABLED', 'Enable Camera Trigger support', 0, None),
    Feature('Camera', 'Camera_MAVLink', 'AP_CAMERA_MAVLINK_ENABLED', 'Enable MAVLink Camera support', 0, 'Camera'),
    Feature('Camera', 'Camera_MAVLinkCamV2', 'AP_CAMERA_MAVLINKCAMV2_ENABLED', 'Enable MAVLink CameraV2 support', 0, 'Camera'),
    Feature('Camera', 'Camera_Mount', 'AP_CAMERA_MOUNT_ENABLED', 'Enable Camera-in-Mount support', 0, 'Camera,MOUNT'),
    Feature('Camera', 'Camera_Relay', 'AP_CAMERA_RELAY_ENABLED', 'Enable Camera Trigger via Relay support', 0, 'Camera,RELAY'),
    Feature('Camera', 'Camera_Servo', 'AP_CAMERA_SERVO_ENABLED', 'Enable Camera Trigger via Servo support', 0, 'Camera'),
    Feature('Camera', 'Camera_Solo', 'AP_CAMERA_SOLOGIMBAL_ENABLED', 'Enable Solo Gimbal support', 0, 'Camera'),
    Feature('Camera', 'Camera_FOV_Status', 'AP_CAMERA_SEND_FOV_STATUS_ENABLED', 'Enable Camera FOV Status send to GCS', 0, 'Camera,MOUNT'),  # noqa: E501

    Feature('Camera', 'RUNCAM', 'HAL_RUNCAM_ENABLED', 'Enable RunCam Control', 0, None),

    Feature('Copter', 'MODE_ZIGZAG', 'MODE_ZIGZAG_ENABLED', 'Enable Mode ZigZag', 0, None),
    Feature('Copter', 'MODE_SYSTEMID', 'MODE_SYSTEMID_ENABLED', 'Enable Mode SystemID', 0, 'Logging'),
    Feature('Copter', 'MODE_SPORT', 'MODE_SPORT_ENABLED', 'Enable Mode Sport', 0, None),
    Feature('Copter', 'MODE_FOLLOW', 'MODE_FOLLOW_ENABLED', 'Enable Mode Follow', 0, 'AC_AVOID'),
    Feature('Copter', 'MODE_TURTLE', 'MODE_TURTLE_ENABLED', 'Enable Mode Turtle', 0, None),
    Feature('Copter', 'MODE_GUIDED_NOGPS', 'MODE_GUIDED_NOGPS_ENABLED', 'Enable Mode Guided NoGPS', 0, None),
    Feature('Copter', 'MODE_FLOWHOLD', 'MODE_FLOWHOLD_ENABLED', 'Enable Mode Flowhold', 0, "OPTICALFLOW"),
    Feature('Copter', 'MODE_FLIP', 'MODE_FLIP_ENABLED', 'Enable Mode Flip', 0, None),
    Feature('Copter', 'MODE_BRAKE', 'MODE_BRAKE_ENABLED', 'Enable Mode Brake', 0, None),

    Feature('Mission', 'MISSION_NAV_PAYLOAD_PLACE', 'AP_MISSION_NAV_PAYLOAD_PLACE_ENABLED', 'Enable handling of NAV_PAYLOAD_PLACE mission items', 0, None),  # noqa
    Feature('Copter', 'AC_PAYLOAD_PLACE_ENABLED', 'AC_PAYLOAD_PLACE_ENABLED', 'Enable Payload Place flight behaviour', 0, 'MISSION_NAV_PAYLOAD_PLACE'),  # noqa

    Feature('Compass', 'AK09916', 'AP_COMPASS_AK09916_ENABLED', 'Enable AK09916 compasses', 1, None),
    Feature('Compass', 'AK8963', 'AP_COMPASS_AK8963_ENABLED', 'Enable AK8963 compasses', 1, None),
    Feature('Compass', 'BMM150', 'AP_COMPASS_BMM150_ENABLED', 'Enable BMM150 compasses', 1, None),
    Feature('Compass', 'EXTERNALAHRS_COMPASS', 'AP_COMPASS_EXTERNALAHRS_ENABLED', 'Enable ExternalAHRS compasses', 0, "AHRS_EXT"),  # noqa
    Feature('Compass', 'HMC5843', 'AP_COMPASS_HMC5843_ENABLED', 'Enable HMC5843 compasses', 1, None),
    Feature('Compass', 'ICM20948', 'AP_COMPASS_ICM20948_ENABLED', 'Enable AK09916 on ICM20948 compasses', 1, "AK09916"),
    Feature('Compass', 'IST8308', 'AP_COMPASS_IST8308_ENABLED', 'Enable IST8308 compasses', 1, None),
    Feature('Compass', 'LIS3MDL', 'AP_COMPASS_LIS3MDL_ENABLED', 'Enable LIS3MDL compasses', 1, None),
    Feature('Compass', 'LSM303D', 'AP_COMPASS_LSM303D_ENABLED', 'Enable LSM303D compasses', 1, None),
    Feature('Compass', 'LSM9DS1', 'AP_COMPASS_LSM9DS1_ENABLED', 'Enable LSM9DS1 compasses', 1, None),
    Feature('Compass', 'MAG3110', 'AP_COMPASS_MAG3110_ENABLED', 'Enable MAG3110 compasses', 1, None),
    Feature('Compass', 'MMC3416', 'AP_COMPASS_MMC3416_ENABLED', 'Enable MMC3416 compasses', 1, None),
    Feature('Compass', 'MMC5XX3', 'AP_COMPASS_MMC5XX3_ENABLED', 'Enable MMC5XX3 compasses', 1, None),
    Feature('Compass', 'QMC5883L', 'AP_COMPASS_QMC5883L_ENABLED', 'Enable QMC5883L compasses', 1, None),
    Feature('Compass', 'RM3100', 'AP_COMPASS_RM3100_ENABLED', 'Enable RM3100 compasses', 1, None),
    Feature('Compass', 'DRONECAN_COMPASS', 'AP_COMPASS_DRONECAN_ENABLED', 'Enable DroneCAN compasses', 0, "DroneCAN"),
    Feature('Compass', 'DRONECAN_COMPASS_HIRES', 'AP_COMPASS_DRONECAN_HIRES_ENABLED', 'Enable DroneCAN HiRes compasses for survey logging', 0, "DroneCAN"), # noqa
    Feature('Compass', 'FixedYawCal', 'AP_COMPASS_CALIBRATION_FIXED_YAW_ENABLED', 'Enable Fixed-Yaw Compass Calibration', 1, None),  # noqa
    Feature('Compass', 'CompassLearn', 'COMPASS_LEARN_ENABLED', 'Enable In-Flight Compass Learning', 1, "FixedYawCal"),

    Feature('Gimbal', 'MOUNT', 'HAL_MOUNT_ENABLED', 'Enable Mount', 0, None),
    Feature('Gimbal', 'ALEXMOS', 'HAL_MOUNT_ALEXMOS_ENABLED', 'Enable Alexmos Gimbal', 0, "MOUNT"),
    Feature('Gimbal', 'GREMSY', 'HAL_MOUNT_GREMSY_ENABLED', 'Enable Gremsy Gimbal', 0, "MOUNT"),
    Feature('Gimbal', 'SERVO', 'HAL_MOUNT_SERVO_ENABLED', 'Enable Servo Gimbal', 0, "MOUNT"),
    Feature('Gimbal', 'SIYI', 'HAL_MOUNT_SIYI_ENABLED', 'Enable Siyi Gimbal', 0, "MOUNT"),
    Feature('Gimbal', 'SOLOGIMBAL', 'HAL_SOLO_GIMBAL_ENABLED', 'Enable Solo Gimbal', 0, "MOUNT"),
    Feature('Gimbal', 'STORM32_MAVLINK', 'HAL_MOUNT_STORM32MAVLINK_ENABLED', 'Enable SToRM32 MAVLink Gimbal', 0, "MOUNT"),
    Feature('Gimbal', 'STORM32_SERIAL', 'HAL_MOUNT_STORM32SERIAL_ENABLED', 'Enable SToRM32 Serial Gimbal', 0, "MOUNT"),
    Feature('Gimbal', 'XACTI', 'HAL_MOUNT_XACTI_ENABLED', 'Enable Xacti Gimbal', 0, "MOUNT,DroneCAN"),
    Feature('Gimbal', 'VIEWPRO', 'HAL_MOUNT_VIEWPRO_ENABLED', 'Enable Viewpro Gimbal', 0, "MOUNT"),

    Feature('VTOL Frame', 'QUAD', 'AP_MOTORS_FRAME_QUAD_ENABLED', 'QUADS(BI,TRI also)', 1, None),
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
    Feature('Payload', 'RELAY', 'AP_RELAY_ENABLED', 'Enable Relay support', 0, None),
    Feature('Payload', 'SERVORELAY_EVENTS', 'AP_SERVORELAYEVENTS_ENABLED', 'Enable Servo/Relay Event support', 0, None),

    Feature('Plane', 'ADVANCED_FAILSAFE', 'AP_ADVANCEDFAILSAFE_ENABLED', 'Enable Advanced Failsafe features', 0, None),
    Feature('Plane', 'QUADPLANE', 'HAL_QUADPLANE_ENABLED', 'Enable QuadPlane support', 0, None),
    Feature('Plane', 'SOARING', 'HAL_SOARING_ENABLED', 'Enable Soaring', 0, None),
    Feature('Plane', 'DEEPSTALL', 'HAL_LANDING_DEEPSTALL_ENABLED', 'Enable Deepstall Landing', 0, None),
    Feature('Plane', 'QAUTOTUNE', 'QAUTOTUNE_ENABLED', 'Enable QuadPlane Autotune mode', 0, "QUADPLANE"),
    Feature('Plane', 'PLANE_BLACKBOX', 'AP_PLANE_BLACKBOX_LOGGING', 'Enable blackbox logging', 0, None),
    Feature('Plane', 'AP_TX_TUNING', 'AP_TUNING_ENABLED', 'Enable TX-based tuning parameter adjustments', 0, None),

    Feature('RC', 'RC_Protocol', 'AP_RCPROTOCOL_ENABLED', "Enable Serial RC Protocol support", 0, None),   # NOQA: E501
    Feature('RC', 'RC_CRSF', 'AP_RCPROTOCOL_CRSF_ENABLED', "Enable CRSF RC Protocol", 0, "RC_Protocol"),   # NOQA: E501
    Feature('RC', 'RC_IBUS', 'AP_RCPROTOCOL_IBUS_ENABLED', "Enable IBus RC Protocol", 0, "RC_Protocol"),   # NOQA: E501
    Feature('RC', 'RC_SBUS', 'AP_RCPROTOCOL_SBUS_ENABLED', "Enable SBUS Protocol", 0, "RC_Protocol"),   # NOQA: E501
    Feature('RC', 'RC_PPMSUM', 'AP_RCPROTOCOL_PPMSUM_ENABLED', "Enable PPMSum Protocol", 0, "RC_Protocol"),   # NOQA: E501
    Feature('RC', 'RC_SRXL', 'AP_RCPROTOCOL_SRXL_ENABLED', "Enable SRXL RC Protocol", 0, "RC_Protocol"),   # NOQA: E501
    Feature('RC', 'RC_SRXL2', 'AP_RCPROTOCOL_SRXL2_ENABLED', "Enable SRXL2 RC Protocol", 0, "RC_Protocol"),   # NOQA: E501
    Feature('RC', 'RC_ST24', 'AP_RCPROTOCOL_ST24_ENABLED', "Enable ST24 Protocol", 0, "RC_Protocol"),   # NOQA: E501
    Feature('RC', 'RC_SUMD', 'AP_RCPROTOCOL_SUMD_ENABLED', "Enable SUMD RC Protocol", 0, "RC_Protocol"),   # NOQA: E501
    Feature('RC', 'RC_GHST', 'AP_RCPROTOCOL_GHST_ENABLED', "Enable Ghost RC Protocol", 0, "RC_Protocol"),   # NOQA: E501
    Feature('RC', 'RC_MAVLINK_RADIO', 'AP_RCPROTOCOL_MAVLINK_RADIO_ENABLED', "Enable MAVLink RC Protocol", 0, "RC_Protocol"),   # NOQA: E501

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

    Feature('Baro', 'BMP085', 'AP_BARO_BMP085_ENABLED', 'Enable BMP085 Barometric Sensor', 1, None),
    Feature('Baro', 'BMP280', 'AP_BARO_BMP280_ENABLED', 'Enable BMP280 Barometric Sensor', 1, None),
    Feature('Baro', 'BMP388', 'AP_BARO_BMP388_ENABLED', 'Enable BMP388 Barometric Sensor', 1, None),
    Feature('Baro', 'BMP581', 'AP_BARO_BMP581_ENABLED', 'Enable BMP581 Barometric Sensor', 1, None),
    Feature('Baro', 'DPS280', 'AP_BARO_DPS280_ENABLED', 'Enable DPS280/DPS310 Barometric Sensor', 1, None),
    Feature('Baro', 'DUMMY', 'AP_BARO_DUMMY_ENABLED', 'Enable DUMMY Barometric Sensor', 0, None),
    Feature('Baro', 'EXTERNALAHRS', 'AP_BARO_EXTERNALAHRS_ENABLED', 'Enable EXTERNALAHRS Barometric Sensor', 0, 'AHRS_EXT'),
    Feature('Baro', 'FBM320', 'AP_BARO_FBM320_ENABLED', 'Enable FBM320 Barometric Sensor', 1, None),
    Feature('Baro', 'ICM20789', 'AP_BARO_ICM20789_ENABLED', 'Enable ICM20789 Barometric Sensor', 1, None),
    Feature('Baro', 'KELLERLD', 'AP_BARO_KELLERLD_ENABLED', 'Enable KELLERLD Barometric Sensor', 1, None),
    Feature('Baro', 'LPS2XH', 'AP_BARO_LPS2XH_ENABLED', 'Enable LPS2XH Barometric Sensor', 1, None),
    Feature('Baro', 'MS56XX', 'AP_BARO_MS56XX_ENABLED', 'Enable MS56XX Barometric Sensor', 1, None),
    Feature('Baro', 'MSP_BARO', 'AP_BARO_MSP_ENABLED', 'Enable MSP Barometric Sensor', 0, 'MSP'),
    Feature('Baro', 'SPL06', 'AP_BARO_SPL06_ENABLED', 'Enable SPL06 Barometric Sensor', 1, None),
    Feature('Baro', 'DRONECAN_BARO', 'AP_BARO_DRONECAN_ENABLED', 'Enable DroneCAN Barometric Sensor', 0, "DroneCAN"),
    Feature('Baro', 'ICP101XX', 'AP_BARO_ICP101XX_ENABLED', 'Enable ICP101XX Barometric Sensor', 0, None),
    Feature('Baro', 'ICP201XX', 'AP_BARO_ICP201XX_ENABLED', 'Enable ICP201XX Barometric Sensor', 0, None),
    Feature('Baro', 'BARO_TEMPCAL', 'AP_TEMPCALIBRATION_ENABLED', 'Enable Baro Temperature Calibration', 0, None),

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

    Feature('Sensors', 'AIRSPEED', 'AP_AIRSPEED_ENABLED', 'Enable Airspeed Sensors', 1, None),    # Default to enabled to not annoy Plane users   # NOQA: E501
    Feature('Sensors', 'BEACON', 'AP_BEACON_ENABLED', 'Enable Beacon', 0, None),
    Feature('Sensors', 'GPS_MOVING_BASELINE', 'GPS_MOVING_BASELINE', 'Enable GPS Moving Baseline', 0, None),
    Feature('Sensors', 'IMU_ON_UART', 'AP_SERIALMANAGER_IMUOUT_ENABLED', 'Enable sending raw IMU data on a serial port', 0, None), # NOQA: E501

    Feature('Other', 'HarmonicNotches', 'AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED', 'Enable InertialSensor Harmonic Notches', 0, None),  # noqa
    Feature('Other', 'GyroFFT', 'HAL_GYROFFT_ENABLED', 'Enable In-Flight Gyro FFT calculations', 0, None),
    Feature('Other', 'NMEA_OUTPUT', 'HAL_NMEA_OUTPUT_ENABLED', 'Enable NMEA Output', 0, None),
    Feature('Other', 'SDCARD_FORMATTING', 'AP_FILESYSTEM_FORMAT_ENABLED', 'Enable formatting of microSD cards', 0, None),
    Feature('Other', 'BOOTLOADER_FLASHING', 'AP_BOOTLOADER_FLASHING_ENABLED', 'Enable Bootloader flashing', 0, "FILESYSTEM_ROMFS"),  # noqa
    Feature('Other', 'SCRIPTING', 'AP_SCRIPTING_ENABLED', 'Enable LUA Scripting', 0, None),
    Feature('Other', 'SLCAN', 'AP_CAN_SLCAN_ENABLED', 'Enable SLCAN serial protocol', 0, None),
    Feature('Other', 'SDCARD_MISSION', 'AP_SDCARD_STORAGE_ENABLED', 'Enable storing mission on microSD cards', 0, None),
    Feature('Other', 'COMPASS_CAL', 'COMPASS_CAL_ENABLED', 'Enable "tumble" compass calibration', 0, None),
    Feature('Other', 'DRONECAN_SERIAL', 'AP_DRONECAN_SERIAL_ENABLED', 'Enable DroneCAN virtual serial ports', 0, "DroneCAN"),
    Feature('Other', 'Buttons', 'HAL_BUTTON_ENABLED', 'Enable Buttons', 0, None),
    Feature('Other', 'Logging', 'HAL_LOGGING_ENABLED', 'Enable Logging', 0, None),
    Feature('Other', 'CUSTOM_ROTATIONS', 'AP_CUSTOMROTATIONS_ENABLED', 'Enable Custom Rotations', 0, None),

    # MAVLink section for mavlink features and/or message handling,
    # rather than for e.g. mavlink-based sensor drivers
    Feature('MAVLink', 'HIGHLAT2', 'HAL_HIGH_LATENCY2_ENABLED', 'Enable HighLatency2 Support', 0, None),
    Feature('MAVLink', 'FENCEPOINT_PROTOCOL', 'AC_POLYFENCE_FENCE_POINT_PROTOCOL_SUPPORT', 'Enable old MAVLink FencePoint protocol', 0, "FENCE"),  # noqa
    Feature('MAVLink', 'RALLYPOINT_PROTOCOL', 'AP_MAVLINK_RALLY_POINT_PROTOCOL_ENABLED', 'Enable old MAVLink RallyPoint protocol', 0, "RALLY"),  # noqa
    Feature('MAVLink', 'MAVLINK_VERSION_REQUEST', 'AP_MAVLINK_AUTOPILOT_VERSION_REQUEST_ENABLED', 'Enable old AUTOPILOT_VERSION_REQUEST mesage', 0, None),  # noqa
    Feature('MAVLink', 'REQUEST_AUTOPILOT_CAPA', 'AP_MAVLINK_MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES_ENABLED', 'Enable old REQUEST_AUTOPILOT_CAPABILITIES command', 0, None),  # noqa
    Feature('MAVLink', 'MAV_MSG_RELAY_STATUS', 'AP_MAVLINK_MSG_RELAY_STATUS_ENABLED', 'Enable sending of RELAY_STATUS message', 0, 'RELAY'),  # noqa
    Feature('MAVLink', 'MAV_MSG_HIL_GPS', 'AP_MAVLINK_MSG_HIL_GPS_ENABLED', 'Enable receiving of HIL_GPS messages', 0, None),  # noqa
    Feature('MAVLink', 'MAV_MSG_MOUNT_CONTROL', 'AP_MAVLINK_MSG_MOUNT_CONTROL_ENABLED', 'Enable handling of deprecated MOUNT_CONTROL message', 0, None),  # noqa
    Feature('MAVLink', 'MAV_MSG_MOUNT_CONFIGURE', 'AP_MAVLINK_MSG_MOUNT_CONFIGURE_ENABLED', 'Enable handling of deprecated MOUNT_CONFIGURE message', 0, None),  # noqa
    Feature('MAVLink', 'AP_MAVLINK_BATTERY2_ENABLED', 'AP_MAVLINK_BATTERY2_ENABLED', 'Enable sending of old BATTERY2 message', 0, None),  # noqa
    Feature('MAVLink', 'MAV_DEVICE_OP', 'AP_MAVLINK_MSG_DEVICE_OP_ENABLED', 'Enable handling of DeviceOp mavlink messages', 0, None),  # noqa
    Feature('MAVLink', 'MAV_SERVO_RELAY', 'AP_MAVLINK_SERVO_RELAY_ENABLED', 'Enable handling of ServoRelay mavlink messages', 0, 'SERVORELAY_EVENTS'),  # noqa
    Feature('MAVLink', 'MAV_MSG_SERIAL_CONTROL', 'AP_MAVLINK_MSG_SERIAL_CONTROL_ENABLED', 'Enable handling of Serial Control mavlink messages', 0, None),  # noqa
    Feature('MAVLink', 'MAVLINK_MSG_MISSION_REQUEST', 'AP_MAVLINK_MSG_MISSION_REQUEST_ENABLED', 'Enable handling of MISSION_REQUEST mavlink messages', 0, None),  # noqa
    Feature('MAVLink', 'AP_MAVLINK_FTP_ENABLED', 'AP_MAVLINK_FTP_ENABLED', 'Enable MAVLink FTP Protocol', 0, None),  # noqa

    Feature('Developer', 'KILL_IMU', 'AP_INERTIALSENSOR_KILL_IMU_ENABLED', 'Allow IMUs to be disabled at runtime', 0, None),
    Feature('Developer', 'CRASHCATCHER', 'AP_CRASHDUMP_ENABLED', 'Enable CrashCatcher', 0, None),

    Feature('GPS Drivers', 'UBLOX', 'AP_GPS_UBLOX_ENABLED', 'Enable u-blox GPS', 1, None),
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
    Feature('GPS Drivers', 'DroneCAN_GPS_Out', 'AP_DRONECAN_SEND_GPS', 'Enable Sending GPS from Autopilot', 0, "DroneCAN"),
    Feature('GPS Drivers', 'GPS_Blending', 'AP_GPS_BLENDED_ENABLED', 'Enable GPS Blending', 0, None),


    Feature('Airspeed Drivers', 'Analog', 'AP_AIRSPEED_ANALOG_ENABLED', 'Enable Analog Airspeed', 0, 'AIRSPEED'),
    Feature('Airspeed Drivers', 'ASP5033', 'AP_AIRSPEED_ASP5033_ENABLED', 'ENABLE ASP5033 AIRSPEED', 0, 'AIRSPEED'),  # NOQA: E501
    Feature('Airspeed Drivers', 'DLVR', 'AP_AIRSPEED_DLVR_ENABLED', 'ENABLE DLVR AIRSPEED', 0, 'AIRSPEED'),
    Feature('Airspeed Drivers', 'MS4525', 'AP_AIRSPEED_MS4525_ENABLED', 'ENABLE MS4525 AIRSPEED', 0, 'AIRSPEED'),
    Feature('Airspeed Drivers', 'MS5525', 'AP_AIRSPEED_MS5525_ENABLED', 'ENABLE MS5525 AIRSPEED', 0, 'AIRSPEED'),
    Feature('Airspeed Drivers', 'MSP_AIRSPEED', 'AP_AIRSPEED_MSP_ENABLED', 'ENABLE MSP AIRSPEED', 0, 'AIRSPEED,MSP,OSD'),
    Feature('Airspeed Drivers', 'NMEA_AIRSPEED', 'AP_AIRSPEED_NMEA_ENABLED', 'ENABLE NMEA AIRSPEED', 0, 'AIRSPEED'),
    Feature('Airspeed Drivers', 'SDP3X', 'AP_AIRSPEED_SDP3X_ENABLED', 'ENABLE SDP3X AIRSPEED', 0, 'AIRSPEED'),
    Feature('Airspeed Drivers', 'DRONECAN_ASPD', 'AP_AIRSPEED_DRONECAN_ENABLED', 'ENABLE DroneCAN AIRSPEED', 0, 'AIRSPEED,DroneCAN'),   # NOQA: E501

    Feature('Actuators', 'Volz', 'AP_VOLZ_ENABLED', 'Enable Volz Protocol', 0, None),
    Feature('Actuators', 'Volz_DroneCAN', 'AP_DRONECAN_VOLZ_FEEDBACK_ENABLED', 'Enable Volz DroneCAN Feedback', 0, "DroneCAN"),
    Feature('Actuators', 'RobotisServo', 'AP_ROBOTISSERVO_ENABLED', 'Enable RobotisServo Protocol', 0, None),
    Feature('Actuators', 'SBUS Output', 'AP_SBUSOUTPUT_ENABLED', 'Enable SBUS Output on serial ports', 0, None),
    Feature('Actuators', 'FETTecOneWire', 'AP_FETTEC_ONEWIRE_ENABLED', 'Enable FETTec OneWire ESCs', 0, None),
    Feature('Actuators', 'KDECAN', 'AP_KDECAN_ENABLED', 'KDE Direct KDECAN ESC', 0, None),
    Feature('Actuators', 'HimarkServo', 'AP_DRONECAN_HIMARK_SERVO_SUPPORT', 'Enable Himark DroneCAN servos', 0, "DroneCAN"),
    Feature('Actuators', 'HobbywingESC', 'AP_DRONECAN_HOBBYWING_ESC_SUPPORT', 'Enable Hobbywing DroneCAN ESCs', 0, "DroneCAN"),

    Feature('Precision Landing', 'PrecLand', 'AC_PRECLAND_ENABLED', 'Enable Precision Landing support', 0, None),
    Feature('Precision Landing', 'PrecLand - Companion', 'AC_PRECLAND_COMPANION_ENABLED', 'Enable Companion-Supported Precision Landing support', 0, "PrecLand"),  # noqa
    Feature('Precision Landing', 'PrecLand - IRLock', 'AC_PRECLAND_IRLOCK_ENABLED', 'Enable IRLock-Supported Precision Landing support', 0, "PrecLand"),  # noqa

    #    Feature('Filesystem', 'FILESYSTEM_ESP32_ENABLED', 'AP_FILESYSTEM_ESP32_ENABLED', 'Enable ESP32 Filesystem', 0, None),
    # Feature('Filesystem', 'FILESYSTEM_FATFS', 'AP_FILESYSTEM_FATFS_ENABLED', 'Enable FATFS Filesystem', 0, None),
    Feature('Filesystem', 'FILESYSTEM_MISSION', 'AP_FILESYSTEM_MISSION_ENABLED', 'Enable @MISSION/ filesystem', 0, None),
    Feature('Filesystem', 'FILESYSTEM_PARAM', 'AP_FILESYSTEM_PARAM_ENABLED', 'Enable @PARAM/ filesystem', 0, None),
    #    Feature('Filesystem', 'FILESYSTEM_POSIX', 'AP_FILESYSTEM_POSIX_ENABLED', 'Enable POSIX filesystem', 0, None),
    Feature('Filesystem', 'FILESYSTEM_ROMFS', 'AP_FILESYSTEM_ROMFS_ENABLED', 'Enable @ROMFS/ filesystem', 0, None),
    Feature('Filesystem', 'FILESYSTEM_SYS', 'AP_FILESYSTEM_SYS_ENABLED', 'Enable @SYS/ filesystem', 0, None),
    Feature('Filesystem', 'APJ_TOOL_PARAMETERS', 'FORCE_APJ_DEFAULT_PARAMETERS', 'Enable apj_tool parameter area', 0, None),

    Feature('Networking', 'PPP Support', 'AP_NETWORKING_BACKEND_PPP', 'Enable PPP networking', 0, None),

    Feature('DroneCAN', 'DroneCAN', 'HAL_ENABLE_DRONECAN_DRIVERS', 'Enable DroneCAN support', 0, None),
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
