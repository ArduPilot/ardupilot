#!/usr/bin/env python

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
    Feature('AHRS', 'VISUALODOM', 'HAL_VISUALODOM_ENABLED', 'Enable Visual Odometry', 0, None),

    Feature('Safety', 'PARACHUTE', 'HAL_PARACHUTE_ENABLED', 'Enable Parachute', 0, None),
    Feature('Safety', 'PROXIMITY', 'HAL_PROXIMITY_ENABLED', 'Enable Proximity', 0, None),
    Feature('Safety', 'AC_OAPATHPLANNER', 'AC_OAPATHPLANNER_ENABLED', 'Enable Object Avoidance Path Planner', 0, None),

    Feature('Battery', 'BATTMON_FUEL', 'AP_BATTMON_FUEL_ENABLE', 'Enable Fuel BatteryMonitor', 0, None),
    Feature('Battery', 'BATTMON_SMBUS', 'AP_BATTMON_SMBUS_ENABLE', 'Enable SMBUS BatteryMonitor', 0, None),
    Feature('Battery', 'BATTMON_INA2XX', 'HAL_BATTMON_INA2XX_ENABLED', 'Enable INA2XX BatteryMonitor', 0, None),

    Feature('Ident', 'ADSB', 'HAL_ADSB_ENABLED', 'Enable ADSB', 0, None),
    Feature('Ident', 'ADSB_SAGETECH', 'HAL_ADSB_SAGETECH_ENABLED', 'Enable SageTech ADSB', 0, 'ADSB'),
    Feature('Ident', 'ADSB_UAVIONIX', 'HAL_ADSB_UAVIONIX_MAVLINK_ENABLED', 'Enable Uavionix ADSB', 0, 'ADSB'),
    Feature('Ident', 'AIS', 'HAL_AIS_ENABLED', 'Enable AIS', 0, None),

    Feature('Telemetry', 'CRSF', 'HAL_CRSF_TELEM_ENABLED', 'Enable CRSF Telemetry', 0, None),
    Feature('Telemetry', 'CRSFText', 'HAL_CRSF_TELEM_TEXT_SELECTION_ENABLED', 'Enable CRSF Text Param Selection', 0, 'CRSF'),
    Feature('Telemetry', 'HIGHLAT2', 'HAL_HIGH_LATENCY2_ENABLED', 'Enable HighLatency2 Support', 0, None),
    Feature('Telemetry', 'HOTT', 'HAL_HOTT_TELEM_ENABLED', 'Enable HOTT Telemetry', 0, None),
    Feature('Telemetry', 'SPEKTRUM', 'HAL_SPEKTRUM_TELEM_ENABLED', 'Enable Spektrum Telemetry', 0, None),

    Feature('MSP', 'MSP', 'HAL_MSP_ENABLED', 'Enable MSP Telemetry and MSP OSD', 0, 'OSD'),
    Feature('MSP', 'MSP_SENSORS', 'HAL_MSP_SENSORS_ENABLED', 'Enable MSP Sensors', 0, 'MSP_GPS,MSP_BARO,MSP_COMPASS,MSP_AIRSPEED,MSP,MSP_OPTICALFLOW,MSP_RANGEFINDER,OSD'),   # NOQA: E501
    Feature('MSP', 'MSP_GPS', 'HAL_MSP_GPS_ENABLED', 'Enable MSP GPS', 0, 'MSP,OSD'),
    Feature('MSP', 'MSP_COMPASS', 'HAL_MSP_COMPASS_ENABLED', 'Enable MSP Compass', 0, 'MSP,OSD'),
    Feature('MSP', 'MSP_BARO', 'HAL_MSP_BARO_ENABLED', 'Enable MSP Baro', 0, 'MSP,OSD'),
    Feature('MSP', 'MSP_AIRSPEED', 'HAL_MSP_AIRSPEED_ENABLED', 'Enable MSP AirSpeed', 0, 'MSP,OSD'),
    Feature('MSP', 'MSP_OPTICALFLOW', 'HAL_MSP_OPTICALFLOW_ENABLED', 'Enable MSP OpticalFlow', 0, 'MSP,OSD'), # also OPTFLOW dep   # NOQA: E501
    Feature('MSP', 'MSP_RANGEFINDER', 'HAL_MSP_RANGEFINDER_ENABLED', 'Enable MSP Rangefinder', 0, 'MSP,OSD,RANGEFINDERS'),
    Feature('MSP', 'MSP_DISPLAYPORT', 'HAL_WITH_MSP_DISPLAYPORT', 'Enable MSP DisplayPort OSD (aka CANVAS MODE)', 0, 'MSP,OSD'),   # NOQA: E501

    Feature('ICE', 'EFI', 'HAL_EFI_ENABLED', 'Enable EFI Monitoring', 0, None),
    Feature('ICE', 'EFI_NMPWU', 'HAL_EFI_NWPWU_ENABLED', 'Enable EFI NMPMU', 0, None),
    Feature('ICE', 'GENERATOR', 'HAL_GENERATOR_ENABLED', 'Enable Generator', 0, None),

    Feature('OSD', 'OSD', 'OSD_ENABLED', 'Enable OSD', 0, None),
    Feature('OSD', 'PLUSCODE', 'HAL_PLUSCODE_ENABLE', 'Enable PlusCode', 0, None),
    Feature('OSD', 'RUNCAM', 'HAL_RUNCAM_ENABLED', 'Enable RunCam', 0, None),
    Feature('OSD', 'SMARTAUDIO', 'HAL_SMARTAUDIO_ENABLED', 'Enable SmartAudio', 0, None),
    Feature('OSD', 'OSD_PARAM', 'OSD_PARAM_ENABLED', 'Enable OSD param', 0, 'OSD'),
    Feature('OSD', 'OSD_SIDEBARS', 'HAL_OSD_SIDEBAR_ENABLE', 'Enable Scrolling Sidebars', 0, 'OSD'),

    Feature('ESC', 'PICCOLOCAN', 'HAL_PICCOLO_CAN_ENABLE', 'Enable PiccoloCAN', 0, None),
    Feature('ESC', 'TORQEEDO', 'HAL_TORQEEDO_ENABLED', 'Enable Torqeedo Motors', 0, None),


    Feature('Mode', 'MODE_ZIGZAG', 'MODE_ZIGZAG_ENABLED', 'Enable Mode ZigZag', 0, None),
    Feature('Mode', 'MODE_SYSTEMID', 'MODE_SYSTEMID_ENABLED', 'Enable Mode SystemID', 0, None),
    Feature('Mode', 'MODE_SPORT', 'MODE_SPORT_ENABLED', 'Enable Mode Sport', 0, None),
    Feature('Mode', 'MODE_FOLLOW', 'MODE_FOLLOW_ENABLED', 'Enable Mode Follow', 0, None),
    Feature('Mode', 'MODE_TURTLE', 'MODE_TURTLE_ENABLED', 'Enable Mode Turtle', 0, None),
    Feature('Mode', 'MODE_GUIDED_NOGPS', 'MODE_GUIDED_NOGPS_ENABLED', 'Enable Mode Guided NoGPS', 0, None),

    Feature('Gimbal', 'MOUNT', 'HAL_MOUNT_ENABLED', 'Enable Mount', 0, None),
    Feature('Gimbal', 'SOLOGIMBAL', 'HAL_SOLO_GIMBAL_ENABLED', 'Enable Solo Gimbal', 0, None),

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

    Feature('Sensors', 'AIRSPEED', 'AP_AIRSPEED_ENABLED', 'Enable Airspeed Sensors', 1, None),    # Default to enabled to not annoy Plane users   # NOQA: E501
    Feature('Sensors', 'BEACON', 'BEACON_ENABLED', 'Enable Beacon', 0, None),
    Feature('Sensors', 'GPS_MOVING_BASELINE', 'GPS_MOVING_BASELINE', 'Enable GPS Moving Baseline', 0, None),
    Feature('Rangefinders', 'RANGEFINDERS', 'AP_RANGEFINDER_ENABLED', "Enable Rangefinders", 0, None),

    Feature('Sensors', 'OPTICALFLOW', 'AP_OPTICALFLOW_ENABLED', 'Enable Optical Flow', 0, None),
    Feature('Sensors', 'OPTICALFLOW_CXOF', 'AP_OPTICALFLOW_CXOF_ENABLED', 'Enable Optical flow CXOF Sensor', 0, "OPTICALFLOW"),
    Feature('Sensors', 'OPTICALFLOW_HEREFLOW', 'AP_OPTICALFLOW_HEREFLOW_ENABLED', 'Enable Optical flow HereFlow Sensor', 0, "OPTICALFLOW"),   # NOQA: E501
    Feature('Sensors', 'OPTICALFLOW_MAV', 'AP_OPTICALFLOW_MAV_ENABLED', 'Enable Optical flow MAVLink Sensor', 0, "OPTICALFLOW"),   # NOQA: E501
    Feature('Sensors', 'OPTICALFLOW_ONBOARD', 'AP_OPTICALFLOW_ONBOARD_ENABLED', 'Enable Optical flow ONBOARD Sensor', 0, "OPTICALFLOW"),   # NOQA: E501
    Feature('Sensors', 'OPTICALFLOW_PX4FLOW', 'AP_OPTICALFLOW_PX4FLOW_ENABLED', 'Enable Optical flow PX4FLOW Sensor', 0, "OPTICALFLOW"),   # NOQA: E501
    Feature('Sensors', 'OPTICALFLOW_PIXART', 'AP_OPTICALFLOW_PIXART_ENABLED', 'Enable Optical flow PIXART Sensor', 0, "OPTICALFLOW"),   # NOQA: E501
    Feature('Sensors', 'OPTICALFLOW_UPFLOW', 'AP_OPTICALFLOW_UPFLOW_ENABLED', 'Enable Optical flow UPFLOW Sensor', 0, "OPTICALFLOW"),   # NOQA: E501

    Feature('Sensors', 'RPM', 'RPM_ENABLED', 'Enable RPM sensors', 0, None),

    Feature('Other', 'DSP', 'HAL_WITH_DSP', 'Enable DSP for In-Flight FFT', 0, None),
    Feature('Other', 'DISPLAY', 'HAL_DISPLAY_ENABLED', 'Enable I2C Displays', 0, None),
    Feature('Other', 'NMEA_OUTPUT', 'HAL_NMEA_OUTPUT_ENABLED', 'Enable NMEA Output', 0, None),
    Feature('Other', 'BARO_WIND_COMP', 'HAL_BARO_WIND_COMP_ENABLED', 'Enable Baro Wind Compensation', 0, None),

    Feature('GPS Drivers', 'SBP2', 'AP_GPS_SBP2_ENABLED', 'Enable SBP2 GPS', 0, 'SBP'),
    Feature('GPS Drivers', 'SBP', 'AP_GPS_SBP_ENABLED', 'Enable SBP GPS', 0, None),
    Feature('GPS Drivers', 'ERB', 'AP_GPS_ERB_ENABLED', 'Enable ERB GPS', 0, None),
    Feature('GPS Drivers', 'GSOF', 'AP_GPS_GSOF_ENABLED', 'Enable GSOF GPS', 0, None),
    Feature('GPS Drivers', 'NMEA', 'AP_GPS_NMEA_ENABLED', 'Enable NMEA GPS', 0, None),
    Feature('GPS Drivers', 'MAV', 'AP_GPS_MAV_ENABLED', 'Enable MAVLink GPS', 0, None),
    Feature('GPS Drivers', 'NOVA', 'AP_GPS_NOVA_ENABLED', 'Enable NOVA GPS', 0, None),
    Feature('GPS Drivers', 'SBF', 'AP_GPS_SBF_ENABLED', 'Enable SBF GPS', 0, None),
    Feature('GPS Drivers', 'SIRF', 'AP_GPS_SIRF_ENABLED', 'Enable SiRF GPS', 0, None),
]

BUILD_OPTIONS.sort(key=lambda x: x.category)
