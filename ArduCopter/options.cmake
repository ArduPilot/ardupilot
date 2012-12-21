#
# This files is used by cmake to present options to the 
# user in the cmake-gui, it can also be used directly to
# set options in the cmake command line.
#
# This file generates APM_Config_cmake.h
# which overrides the APM_Config.h file. When disributing
# to the Arduino IDE user. APM_Confg_cmake.h could be copied to
# APM_Config.h, but this is not necessary. The 
# advantage would be that the Arduino user would have 
# a more up-to-date/ complete list of options and the developers
# using cmake have a nice gui/ command-line interface.
# 

apm_option("APM_PROGRAMMING_PORT" TYPE STRING
    DESCRIPTION "Programming upload port?"
    DEFAULT "/dev/ttyACM0")

apm_option("CONFIG_APM_HARDWARE" TYPE STRING
    DESCRIPTION "APM Hardware?" 
    OPTIONS "APM_HARDWARE_APM2" "APM_HARDWARE_APM1"
    DEFAULT "APM_HARDWARE_APM2")

apm_option("APM2_BETA_HARDWARE" TYPE BOOL DEFINE_ONLY
    DESCRIPTION "Is this an APM 2.0 Beta board?" 
    DEFAULT OFF)

apm_option("APM_PROCESSOR" TYPE STRING
    DESCRIPTION "ArduPilotMega processor (2560 for APM2 and later APM1)?" 
    DEFAULT "mega2560"
    OPTIONS "mega" "mega2560")

#apm_option("CLI_SLIDER_ENABLED" TYPE BOOL
    #DESCRIPTION "Enable command line interface switch?" 
    #DEFAULT OFF)

apm_option("LOGGING_ENABLED" TYPE BOOL
    DESCRIPTION "Enable logging?" 
    DEFAULT OFF)

apm_option("GPS_PROTOCOL" TYPE STRING
    DESCRIPTION "GPS protocol?"
    DEFAULT "GPS_PROTOCOL_AUTO" 
    OPTIONS 
        "GPS_PROTOOCL_NONE"
        "GPS_PROTOCOL_AUTO"
        "GPS_PROTOCOL_NONE"
        "GPS_PROTOCOL_IMU"
        "GPS_PROTOCOL_MTK"
        "GPS_PROTOCOL_MTK19"
        "GPS_PROTOCOL_UBLOX"
        "GPS_PROTOCOL_SIRF"
        "GPS_PROTOCOL_NMEA")

apm_option("AIRSPEED_SENSOR" TYPE BOOL
    DESCRIPTION "Enable airspeed sensor?"
    DEFAULT OFF)

apm_option("PITOT_ENABLED" TYPE BOOL
    DESCRIPTION "Enable pitot static system?"
    DEFAULT OFF)

apm_option("SONAR_ENABLED" TYPE BOOL
    DESCRIPTION "Enable sonar?"
    DEFAULT OFF)

apm_option("AIRSPEED_RATIO" TYPE STRING ADVANCED
    DESCRIPTION "Airspeed ratio?"
    DEFAULT "1.9936")

#apm_option("MAGNETOMETER" TYPE BOOL
    #DESCRIPTION "Enable airspeed sensor?"
    #DEFAULT OFF)

#apm_option("MAG_ORIENTATION" TYPE STRING ADVANCED
    #DESCRIPTION "Magnetometer orientation?" 
    #DEFAULT "AP_COMPASS_COMPONENTS_DOWN_PINS_FORWARD"
    #OPTIONS 
        #"AP_COMPASS_COMPONENTS_DOWN_PINS_FORWARD"
        #"AP_COMPASS_COMPONENTS_DOWN_PINS_BACK"
        #"AP_COMPASS_COMPONENTS_UP_PINS_FORWARD"
        #"AP_COMPASS_COMPONENTS_UP_PINS_BACK")

apm_option("HIL_MODE" TYPE STRING
    DESCRIPTION "Hardware-in-the-loop- mode?"
    DEFAULT "HIL_MODE_DISABLED"
    OPTIONS 
        "HIL_MODE_DISABLED"
        "HIL_MODE_ATTITUDE"
        "HIL_MODE_SENSORS")

apm_option("MAV_SYSTEM_ID" TYPE STRING ADVANCED
    DESCRIPTION "MAVLink System ID?"
    DEFAULT "1")

apm_option("SERIAL0_BAUD" TYPE STRING ADVANCED
    DESCRIPTION "Serial 0 baudrate?"
    DEFAULT "115200" 
    OPTIONS "57600" "115200") 

apm_option("SERIAL3_BAUD" TYPE STRING ADVANCED
    DESCRIPTION "Serial 3 baudrate?"
    DEFAULT "57600" 
    OPTIONS "57600" "115200") 

apm_option("BATTERY_EVENT" TYPE BOOL ADVANCED
    DESCRIPTION "Enable low voltage/ high discharge warnings?"
    DEFAULT OFF)

apm_option("LOW_VOLTAGE" TYPE STRING ADVANCED
    DESCRIPTION "Voltage to consider low (volts)?"
    DEFAULT "9.6")

apm_option("VOLT_DIV_RATIO" TYPE STRING ADVANCED
    DESCRIPTION "Voltage division ratio?"
    DEFAULT "3.56")

apm_option("CUR_AMPS_PER_VOLT" TYPE STRING ADVANCED
    DESCRIPTION "Current amps/volt?"
    DEFAULT "27.32")

apm_option("CUR_AMPS_OFFSET" TYPE STRING ADVANCED
    DESCRIPTION "Current amps offset?"
    DEFAULT "0.0")

# arducopter specific
apm_option("FRAME_CONFIG" TYPE STRING
    DESCRIPTION "Vehicle type?"
    DEFAULT "QUAD_FRAME"
    OPTIONS
        "QUAD_FRAME"
        "TRI_FRAME"
        "HEXA_FRAME"
        "Y6_FRAME"
        "OCTA_FRAME"
        "OCTA_QUAD_FRAME"
        "HELI_FRAME"
    )

apm_option("FRAME_ORIENTATION" TYPE STRING
    DESCRIPTION "Vehicle type?"
    DEFAULT "PLUS_FRAME"
    OPTIONS
	    "PLUS_FRAME"
	    "X_FRAME"
	    "V_FRAME"
    )

apm_option("CH7_OPTION" TYPE STRING
    DESCRIPTION "Channel 7 option?"
    DEFAULT "CH7_SAVE_WP" 
    OPTIONS 
        "CH7_DO_NOTHING"
        "CH7_DO_NOTHING"
        "CH7_FLIP"
        "CH7_SIMPLE_MODE"
        "CH7_RTL"
        "CH7_SAVE_TRIM"
        "CH7_DO_NOTHING"
        "CH7_SAVE_WP"
        "CH7_DO_NOTHING"
        "CH7_CAMERA_TRIGGER")

apm_option("ACCEL_ALT_HOLD" TYPE BOOL ADVANCED
    DESCRIPTION "Disabled by default, work in progress."
    DEFAULT OFF)
