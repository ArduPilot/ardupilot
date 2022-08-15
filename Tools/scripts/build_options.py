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
    Feature('AHRS', 'TEMPCAL', 'HAL_INS_TEMPERATURE_CAL_ENABLE', 'Enable IMU Temperature Calibration', 0, None),
    Feature('AHRS', 'VISUALODOM', 'HAL_VISUALODOM_ENABLED', 'Enable Visual Odometry', 0, 'EKF3_EXTNAV'),
    Feature('AHRS', 'EKF3_EXTNAV', 'EK3_FEATURE_EXTERNAL_NAV', 'Enable External Navigation for EKF3', 0, None),

    Feature('Safety', 'PARACHUTE', 'HAL_PARACHUTE_ENABLED', 'Enable Parachute', 0, None),
    Feature('Safety', 'FENCE', 'AP_FENCE_ENABLED', 'Enable Geofence', 2, None),
    Feature('Safety', 'PROXIMITY', 'HAL_PROXIMITY_ENABLED', 'Enable Proximity', 0, None),
    Feature('Safety', 'AC_AVOID', 'AC_AVOID_ENABLED', 'Enable Avoidance', 0, 'FENCE'),
    Feature('Safety', 'AC_OAPATHPLANNER', 'AC_OAPATHPLANNER_ENABLED', 'Enable Object Avoidance Path Planner', 0, 'FENCE'),

    Feature('Battery', 'BATTMON_FUELFLOW', 'AP_BATTMON_FUELFLOW_ENABLE', 'Enable Fuel Flow BatteryMonitor', 0, None),
    Feature('Battery', 'BATTMON_FUELLEVEL_PWM', 'AP_BATTMON_FUELLEVEL_PWM_ENABLE', 'Enable Flow Level PWM BattryMonitor', 0, None),  # noqa: E501
    Feature('Battery', 'BATTMON_FUELLEVEL_ANALOG', 'AP_BATTMON_FUELLEVEL_ANALOG_ENABLE', 'Enable Flow Level Analog BattryMonitor', 0, None),  # noqa: E501
    Feature('Battery', 'BATTMON_SMBUS', 'AP_BATTMON_SMBUS_ENABLE', 'Enable SMBUS BatteryMonitor', 0, None),
    Feature('Battery', 'BATTMON_INA2XX', 'HAL_BATTMON_INA2XX_ENABLED', 'Enable INA2XX BatteryMonitor', 0, None),

    Feature('Ident', 'ADSB', 'HAL_ADSB_ENABLED', 'Enable ADSB', 0, None),
    Feature('Ident', 'ADSB_SAGETECH', 'HAL_ADSB_SAGETECH_ENABLED', 'Enable SageTech ADSB', 0, 'ADSB'),
    Feature('Ident', 'ADSB_SAGETECH_MXS', 'HAL_ADSB_SAGETECH_MXS_ENABLED', 'Enable SageTech MXS ADSB', 0, 'ADSB'),
    Feature('Ident', 'ADSB_UAVIONIX', 'HAL_ADSB_UAVIONIX_MAVLINK_ENABLED', 'Enable Uavionix ADSB', 0, 'ADSB'),
    Feature('Ident', 'AIS', 'AP_AIS_ENABLED', 'Enable AIS', 0, None),

    Feature('Telemetry', 'CRSF', 'HAL_CRSF_TELEM_ENABLED', 'Enable CRSF Telemetry', 0, None),
    Feature('Telemetry', 'CRSFText', 'HAL_CRSF_TELEM_TEXT_SELECTION_ENABLED', 'Enable CRSF Text Param Selection', 0, 'CRSF'),
    Feature('Telemetry', 'HIGHLAT2', 'HAL_HIGH_LATENCY2_ENABLED', 'Enable HighLatency2 Support', 0, None),
    Feature('Telemetry', 'HOTT', 'HAL_HOTT_TELEM_ENABLED', 'Enable HOTT Telemetry', 0, None),
    Feature('Telemetry', 'SPEKTRUM', 'HAL_SPEKTRUM_TELEM_ENABLED', 'Enable Spektrum Telemetry', 0, None),
    Feature('Telemetry', 'LTM', 'AP_LTM_TELEM_ENABLED', 'Enable LTM Telemetry', 0, None),

    Feature('MSP', 'MSP', 'HAL_MSP_ENABLED', 'Enable MSP Telemetry and MSP OSD', 0, 'OSD'),
    Feature('MSP', 'MSP_SENSORS', 'HAL_MSP_SENSORS_ENABLED', 'Enable MSP Sensors', 0, 'MSP_GPS,MSP_BARO,MSP_COMPASS,MSP_AIRSPEED,MSP,MSP_OPTICALFLOW,MSP_RANGEFINDER,OSD'),   # NOQA: E501
    Feature('MSP', 'MSP_GPS', 'HAL_MSP_GPS_ENABLED', 'Enable MSP GPS', 0, 'MSP,OSD'),
    Feature('MSP', 'MSP_COMPASS', 'HAL_MSP_COMPASS_ENABLED', 'Enable MSP Compass', 0, 'MSP,OSD'),
    Feature('MSP', 'MSP_OPTICALFLOW', 'HAL_MSP_OPTICALFLOW_ENABLED', 'Enable MSP OpticalFlow', 0, 'MSP,OSD'), # also OPTFLOW dep   # NOQA: E501
    Feature('MSP', 'MSP_RANGEFINDER', 'HAL_MSP_RANGEFINDER_ENABLED', 'Enable MSP Rangefinder', 0, 'MSP,OSD,RANGEFINDERS'),
    Feature('MSP', 'MSP_DISPLAYPORT', 'HAL_WITH_MSP_DISPLAYPORT', 'Enable MSP DisplayPort OSD (aka CANVAS MODE)', 0, 'MSP,OSD'),   # NOQA: E501

    Feature('ICE', 'ICE Engine', 'AP_ICENGINE_ENABLED', 'Enable Internal Combustion Engine support', 0, None),
    Feature('ICE', 'EFI', 'HAL_EFI_ENABLED', 'Enable EFI Monitoring', 0, None),
    Feature('ICE', 'EFI_NMPWU', 'HAL_EFI_NWPWU_ENABLED', 'Enable EFI NMPMU', 0, None),
    Feature('ICE', 'GENERATOR', 'HAL_GENERATOR_ENABLED', 'Enable Generator', 0, None),

    Feature('OSD', 'OSD', 'OSD_ENABLED', 'Enable OSD', 0, None),
    Feature('OSD', 'PLUSCODE', 'HAL_PLUSCODE_ENABLE', 'Enable PlusCode', 0, 'OSD'),
    Feature('OSD', 'OSD_PARAM', 'OSD_PARAM_ENABLED', 'Enable OSD param', 0, 'OSD'),
    Feature('OSD', 'OSD_SIDEBARS', 'HAL_OSD_SIDEBAR_ENABLE', 'Enable Scrolling Sidebars', 0, 'OSD'),

    Feature('VTX', 'SMARTAUDIO', 'HAL_SMARTAUDIO_ENABLED', 'Enable SmartAudio VTX Contol', 0, None),
    Feature('VTX', 'TRAMP', 'AP_TRAMP_ENABLED', 'Enable IRC Tramp VTX Control', 0, None),

    Feature('ESC', 'PICCOLOCAN', 'HAL_PICCOLO_CAN_ENABLE', 'Enable PiccoloCAN', 0, None),
    Feature('ESC', 'TORQEEDO', 'HAL_TORQEEDO_ENABLED', 'Enable Torqeedo Motors', 0, None),

    Feature('Camera', 'RUNCAM', 'HAL_RUNCAM_ENABLED', 'Enable RunCam Control', 0, None),

    Feature('Mode', 'MODE_ZIGZAG', 'MODE_ZIGZAG_ENABLED', 'Enable Mode ZigZag', 0, None),
    Feature('Mode', 'MODE_SYSTEMID', 'MODE_SYSTEMID_ENABLED', 'Enable Mode SystemID', 0, None),
    Feature('Mode', 'MODE_SPORT', 'MODE_SPORT_ENABLED', 'Enable Mode Sport', 0, None),
    Feature('Mode', 'MODE_FOLLOW', 'MODE_FOLLOW_ENABLED', 'Enable Mode Follow', 0, None),
    Feature('Mode', 'MODE_TURTLE', 'MODE_TURTLE_ENABLED', 'Enable Mode Turtle', 0, None),
    Feature('Mode', 'MODE_GUIDED_NOGPS', 'MODE_GUIDED_NOGPS_ENABLED', 'Enable Mode Guided NoGPS', 0, None),
    Feature('Mode', 'MODE_FLOWHOLD', 'MODE_FLOWHOLD_ENABLED', 'Enable Mode Flowhold', 0, "OPTICALFLOW"),

    Feature('Gimbal', 'MOUNT', 'HAL_MOUNT_ENABLED', 'Enable Mount', 0, None),
    Feature('Gimbal', 'ALEXMOS', 'HAL_MOUNT_ALEXMOS_ENABLED', 'Enable Alexmos Gimbal', 0, "MOUNT"),
    Feature('Gimbal', 'GREMSY', 'HAL_MOUNT_GREMSY_ENABLED', 'Enable Gremsy Gimbal', 0, "MOUNT"),
    Feature('Gimbal', 'SERVO', 'HAL_MOUNT_SERVO_ENABLED', 'Enable Servo Gimbal', 0, "MOUNT"),
    Feature('Gimbal', 'SOLOGIMBAL', 'HAL_SOLO_GIMBAL_ENABLED', 'Enable Solo Gimbal', 0, "MOUNT"),
    Feature('Gimbal', 'STORM32_MAVLINK', 'HAL_MOUNT_STORM32MAVLINK_ENABLED', 'Enable SToRM32 MAVLink Gimbal', 0, "MOUNT"),
    Feature('Gimbal', 'STORM32_SERIAL', 'HAL_MOUNT_STORM32SERIAL_ENABLED', 'Enable SToRM32 Serial Gimbal', 0, "MOUNT"),

    Feature('VTOL Frame', 'QUAD', 'AP_MOTORS_FRAME_QUAD_ENABLED', 'QUADS(BI,TRI also)', 1, None),
    Feature('VTOL Frame', 'HEXA', 'AP_MOTORS_FRAME_HEXA_ENABLED', 'HEXA', 0, None),
    Feature('VTOL Frame', 'OCTA', 'AP_MOTORS_FRAME_OCTA_ENABLED', 'OCTA', 0, None),
    Feature('VTOL Frame', 'DECA', 'AP_MOTORS_FRAME_DECA_ENABLED', 'DECA', 0, None),
    Feature('VTOL Frame', 'DODECAHEXA', 'AP_MOTORS_FRAME_DODECAHEXA_ENABLED', 'DODECAHEXA', 0, None),
    Feature('VTOL Frame', 'Y6', 'AP_MOTORS_FRAME_Y6_ENABLED', 'Y6', 0, None),
    Feature('VTOL Frame', 'OCTAQUAD', 'AP_MOTORS_FRAME_OCTAQUAD_ENABLED', 'OCTAQUAD', 0, None),

    Feature('Payload', 'GRIPPER', 'GRIPPER_ENABLED', 'Enable Gripper', 0, None),
    Feature('Payload', 'SPRAYER', 'HAL_SPRAYER_ENABLED', 'Enable Sprayer', 0, None),
    Feature('Payload', 'LANDING_GEAR', 'LANDING_GEAR_ENABLED', 'Enable Landing Gear', 0, None),
    Feature('Payload', 'WINCH', 'WINCH_ENABLED', 'Enable Winch', 0, None),

    Feature('Plane', 'QUADPLANE', 'HAL_QUADPLANE_ENABLED', 'Enable QuadPlane support', 0, None),
    Feature('Plane', 'SOARING', 'HAL_SOARING_ENABLED', 'Enable Soaring', 0, None),
    Feature('Plane', 'DEEPSTALL', 'HAL_LANDING_DEEPSTALL_ENABLED', 'Enable Deepstall Landing', 0, None),

    Feature('Rangefinder', 'RANGEFINDER', 'AP_RANGEFINDER_ENABLED', "Enable Rangefinders", 0, None),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_ANALOG', 'AP_RANGEFINDER_ANALOG_ENABLED', "Enable Rangefinder - Analog", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_BBB_PRU', 'AP_RANGEFINDER_BBB_PRU_ENABLED', "Enable Rangefinder - BBB PRU", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_BEBOP', 'AP_RANGEFINDER_BEBOP_ENABLED', "Enable Rangefinder - Bebop", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_BENEWAKE_CAN', 'AP_RANGEFINDER_BENEWAKE_CAN_ENABLED', "Enable Rangefinder - Benewake (CAN)", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_BENEWAKE_TF02', 'AP_RANGEFINDER_BENEWAKE_TF02_ENABLED', "Enable Rangefinder - Benewake -TF02", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_BENEWAKE_TF03', 'AP_RANGEFINDER_BENEWAKE_TF03_ENABLED', "Enable Rangefinder - Benewake - TF03", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_BENEWAKE_TFMINI', 'AP_RANGEFINDER_BENEWAKE_TFMINI_ENABLED', "Enable Rangefinder - Benewake - TFMini", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_BENEWAKE_TFMINIPLUS', 'AP_RANGEFINDER_BENEWAKE_TFMINIPLUS_ENABLED', "Enable Rangefinder - Benewake - TFMiniPlus", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_BLPING', 'AP_RANGEFINDER_BLPING_ENABLED', "Enable Rangefinder - BLPing", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_GYUS42V2', 'AP_RANGEFINDER_GYUS42V2_ENABLED', "Enable Rangefinder - GYUS42V2", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_HC_SR04', 'AP_RANGEFINDER_HC_SR04_ENABLED', "Enable Rangefinder - HC_SR04", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_LANBAO', 'AP_RANGEFINDER_LANBAO_ENABLED', "Enable Rangefinder - Lanbao", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_LEDDARONE', 'AP_RANGEFINDER_LEDDARONE_ENABLED', "Enable Rangefinder - LeddarOne", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_LEDDARVU8', 'AP_RANGEFINDER_LEDDARVU8_ENABLED', "Enable Rangefinder - LeddarVU8", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_LIGHTWARE_SERIAL', 'AP_RANGEFINDER_LIGHTWARE_SERIAL_ENABLED', "Enable Rangefinder - Lightware (serial)", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_LWI2C', 'AP_RANGEFINDER_LWI2C_ENABLED', "Enable Rangefinder - Lightware (i2c)", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_MAVLINK', 'AP_RANGEFINDER_MAVLINK_ENABLED', "Enable Rangefinder - MAVLink", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_MAXBOTIX_SERIAL', 'AP_RANGEFINDER_MAXBOTIX_SERIAL_ENABLED', "Enable Rangefinder - MaxBotix (serial)", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_MAXSONARI2CXL', 'AP_RANGEFINDER_MAXSONARI2CXL_ENABLED', "Enable Rangefinder - MaxSonarI2CXL", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_NMEA', 'AP_RANGEFINDER_NMEA_ENABLED', "Enable Rangefinder - NMEA", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_PULSEDLIGHTLRF', 'AP_RANGEFINDER_PULSEDLIGHTLRF_ENABLED', "Enable Rangefinder - PulsedLightLRF", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_PWM', 'AP_RANGEFINDER_PWM_ENABLED', "Enable Rangefinder - PWM", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_SIM', 'AP_RANGEFINDER_SIM_ENABLED', "Enable Rangefinder - SIM", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_TRI2C', 'AP_RANGEFINDER_TRI2C_ENABLED', "Enable Rangefinder - TeraRangerI2C", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_TR_SERIAL', 'AP_RANGEFINDER_TERARANGER_SERIAL_ENABLED', "Enable Rangefinder - TeraRanger Serial", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_UAVCAN', 'AP_RANGEFINDER_UAVCAN_ENABLED', "Enable Rangefinder - UAVCAN", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_USD1_CAN', 'AP_RANGEFINDER_USD1_CAN_ENABLED', "Enable Rangefinder - USD1 (CAN)", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_USD1_SERIAL', 'AP_RANGEFINDER_USD1_SERIAL_ENABLED', "Enable Rangefinder - USD1 (SERIAL)", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_VL53L0X', 'AP_RANGEFINDER_VL53L0X_ENABLED', "Enable Rangefinder - VL53L0X", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_VL53L1X', 'AP_RANGEFINDER_VL53L1X_ENABLED', "Enable Rangefinder - VL53L1X", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_WASP', 'AP_RANGEFINDER_WASP_ENABLED', "Enable Rangefinder - Wasp", 0, "RANGEFINDER"),   # NOQA: E501

    Feature('Sensors', 'OPTICALFLOW', 'AP_OPTICALFLOW_ENABLED', 'Enable Optical Flow', 0, None),
    Feature('Sensors', 'OPTICALFLOW_CXOF', 'AP_OPTICALFLOW_CXOF_ENABLED', 'Enable Optical flow CXOF Sensor', 0, "OPTICALFLOW"),
    Feature('Sensors', 'OPTICALFLOW_HEREFLOW', 'AP_OPTICALFLOW_HEREFLOW_ENABLED', 'Enable Optical flow HereFlow Sensor', 0, "OPTICALFLOW"),   # NOQA: E501
    Feature('Sensors', 'OPTICALFLOW_MAV', 'AP_OPTICALFLOW_MAV_ENABLED', 'Enable Optical flow MAVLink Sensor', 0, "OPTICALFLOW"),   # NOQA: E501
    Feature('Sensors', 'OPTICALFLOW_ONBOARD', 'AP_OPTICALFLOW_ONBOARD_ENABLED', 'Enable Optical flow ONBOARD Sensor', 0, "OPTICALFLOW"),   # NOQA: E501
    Feature('Sensors', 'OPTICALFLOW_PX4FLOW', 'AP_OPTICALFLOW_PX4FLOW_ENABLED', 'Enable Optical flow PX4FLOW Sensor', 0, "OPTICALFLOW"),   # NOQA: E501
    Feature('Sensors', 'OPTICALFLOW_PIXART', 'AP_OPTICALFLOW_PIXART_ENABLED', 'Enable Optical flow PIXART Sensor', 0, "OPTICALFLOW"),   # NOQA: E501
    Feature('Sensors', 'OPTICALFLOW_UPFLOW', 'AP_OPTICALFLOW_UPFLOW_ENABLED', 'Enable Optical flow UPFLOW Sensor', 0, "OPTICALFLOW"),   # NOQA: E501

    Feature('Sensors', 'BMP085', 'AP_BARO_BMP085_ENABLED', 'Enable BMP085 Barometric Sensor', 1, None),
    Feature('Sensors', 'BMP280', 'AP_BARO_BMP280_ENABLED', 'Enable BMP280 Barometric Sensor', 1, None),
    Feature('Sensors', 'BMP388', 'AP_BARO_BMP388_ENABLED', 'Enable BMP388 Barometric Sensor', 1, None),
    Feature('Sensors', 'DPS280', 'AP_BARO_DPS280_ENABLED', 'Enable DPS280 Barometric Sensor', 1, None),
    Feature('Sensors', 'DUMMY', 'AP_BARO_DUMMY_ENABLED', 'Enable DUMMY Barometric Sensor', 0, None),
    Feature('Sensors', 'EXTERNALAHRS', 'AP_BARO_EXTERNALAHRS_ENABLED', 'Enable EXTERNALAHRS Barometric Sensor', 0, None),
    Feature('Sensors', 'FBM320', 'AP_BARO_FBM320_ENABLED', 'Enable FBM320 Barometric Sensor', 1, None),
    Feature('Sensors', 'ICM20789', 'AP_BARO_ICM20789_ENABLED', 'Enable ICM20789 Barometric Sensor', 1, None),
    Feature('Sensors', 'KELLERLD', 'AP_BARO_KELLERLD_ENABLED', 'Enable KELLERLD Barometric Sensor', 1, None),
    Feature('Sensors', 'LPS2XH', 'AP_BARO_LPS2XH_ENABLED', 'Enable LPS2XH Barometric Sensor', 1, None),
    Feature('Sensors', 'MS56XX', 'AP_BARO_MS56XX_ENABLED', 'Enable MS56XX Barometric Sensor', 1, None),
    Feature('Sensors', 'MSP_BARO', 'AP_BARO_MSP_ENABLED', 'Enable MSP Barometric Sensor', 0, 'MSP'),
    Feature('Sensors', 'SPL06', 'AP_BARO_SPL06_ENABLED', 'Enable SPL06 Barometric Sensor', 1, None),
    Feature('Sensors', 'UAVCAN_BARO', 'AP_BARO_UAVCAN_ENABLED', 'Enable UAVCAN Barometric Sensor', 0, None),
    Feature('Sensors', 'ICP101XX', 'AP_BARO_ICP101XX_ENABLED', 'Enable ICP101XX Barometric Sensor', 0, None),
    Feature('Sensors', 'ICP201XX', 'AP_BARO_ICP201XX_ENABLED', 'Enable ICP201XX Barometric Sensor', 0, None),

    Feature('Sensors', 'RPM', 'RPM_ENABLED', 'Enable RPM sensors', 0, None),
    Feature('Sensors', 'AIRSPEED', 'AP_AIRSPEED_ENABLED', 'Enable Airspeed Sensors', 1, None),    # Default to enabled to not annoy Plane users   # NOQA: E501
    Feature('Sensors', 'BEACON', 'BEACON_ENABLED', 'Enable Beacon', 0, None),
    Feature('Sensors', 'GPS_MOVING_BASELINE', 'GPS_MOVING_BASELINE', 'Enable GPS Moving Baseline', 0, None),


    Feature('Other', 'DSP', 'HAL_WITH_DSP', 'Enable DSP for In-Flight FFT', 0, None),
    Feature('Other', 'DISPLAY', 'HAL_DISPLAY_ENABLED', 'Enable I2C Displays', 0, None),
    Feature('Other', 'NMEA_OUTPUT', 'HAL_NMEA_OUTPUT_ENABLED', 'Enable NMEA Output', 0, None),
    Feature('Other', 'BARO_WIND_COMP', 'HAL_BARO_WIND_COMP_ENABLED', 'Enable Baro Wind Compensation', 0, None),

    Feature('GPS Drivers', 'UBLOX', 'AP_GPS_UBLOX_ENABLED', 'Enable u-blox GPS', 1, None),
    Feature('GPS Drivers', 'SBP2', 'AP_GPS_SBP2_ENABLED', 'Enable SBP2 GPS', 0, 'SBP'),
    Feature('GPS Drivers', 'SBP', 'AP_GPS_SBP_ENABLED', 'Enable SBP GPS', 0, None),
    Feature('GPS Drivers', 'ERB', 'AP_GPS_ERB_ENABLED', 'Enable ERB GPS', 0, None),
    Feature('GPS Drivers', 'GSOF', 'AP_GPS_GSOF_ENABLED', 'Enable GSOF GPS', 0, None),
    Feature('GPS Drivers', 'NMEA', 'AP_GPS_NMEA_ENABLED', 'Enable NMEA GPS', 0, None),
    Feature('GPS Drivers', 'MAV', 'AP_GPS_MAV_ENABLED', 'Enable MAVLink GPS', 0, None),
    Feature('GPS Drivers', 'NOVA', 'AP_GPS_NOVA_ENABLED', 'Enable NOVA GPS', 0, None),
    Feature('GPS Drivers', 'SBF', 'AP_GPS_SBF_ENABLED', 'Enable SBF GPS', 0, None),
    Feature('GPS Drivers', 'SIRF', 'AP_GPS_SIRF_ENABLED', 'Enable SiRF GPS', 0, None),

    Feature('Airspeed Drivers', 'Analog', 'AP_AIRSPEED_ANALOG_ENABLED', 'Enable Analog Airspeed', 0, 'AIRSPEED'),
    Feature('Airspeed Drivers', 'ASP5033', 'AP_AIRSPEED_ASP5033_ENABLED', 'ENABLE ASP5033 AIRSPEED', 0, 'AIRSPEED'),  # NOQA: E501
    Feature('Airspeed Drivers', 'DLVR', 'AP_AIRSPEED_DLVR_ENABLED', 'ENABLE DLVR AIRSPEED', 0, 'AIRSPEED'),
    Feature('Airspeed Drivers', 'MS4525', 'AP_AIRSPEED_MS4525_ENABLED', 'ENABLE MS4525 AIRSPEED', 0, 'AIRSPEED'),
    Feature('Airspeed Drivers', 'MS5525', 'AP_AIRSPEED_MS5525_ENABLED', 'ENABLE MS5525 AIRSPEED', 0, 'AIRSPEED'),
    Feature('Airspeed Drivers', 'MSP_ARSPD', 'AP_AIRSPEED_MSP_ENABLED', 'ENABLE MSP AIRSPEED', 0, 'AIRSPEED,MSP,OSD'),
    Feature('Airspeed Drivers', 'NMEA', 'AP_AIRSPEED_NMEA_ENABLED', 'ENABLE NMEA AIRSPEED', 0, 'AIRSPEED'),
    Feature('Airspeed Drivers', 'SDP3X', 'AP_AIRSPEED_SDP3X_ENABLED', 'ENABLE SDP3X AIRSPEED', 0, 'AIRSPEED'),
    Feature('Airspeed Drivers', 'UAVCAN_ASPD', 'AP_AIRSPEED_UAVCAN_ENABLED', 'ENABLE UAVCAN AIRSPEED', 0, 'AIRSPEED'),   # NOQA: E501

    Feature('Actuators', 'Volz', 'AP_VOLZ_ENABLED', 'Enable Volz Protocol', 0, None),
    Feature('Actuators', 'RobotisServo', 'AP_ROBOTISSERVO_ENABLED', 'Enable RobotisServo Protocol', 0, None),
    Feature('Actuators', 'FETTecOneWire', 'AP_FETTEC_ONEWIRE_ENABLED', 'Enable FETTec OneWire ESCs', 0, None),
]

BUILD_OPTIONS.sort(key=lambda x: x.category)
