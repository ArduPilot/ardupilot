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
        "GPS_PROTOCOL_MTK16"
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

apm_option("HIGH_DISCHARGE" TYPE STRING ADVANCED
    DESCRIPTION "What to consider high discharge rate (milliamps)?"
    DEFAULT "1760")

apm_option("INPUT_VOLTAGE" TYPE STRING ADVANCED
    DESCRIPTION "Voltage measured at the process (V)? (affects ADC measurements)"
    DEFAULT "4.68")

set(RADIO_CHANNELS "1" "2" "3" "4" "5" "6" "7" "8")
apm_option("FLIGHT_MODE_CHANNEL" TYPE STRING ADVANCED
    DESCRIPTION "The radio channel to control the flight mode."
    DEFAULT "8"
    OPTIONS ${RADIO_CHANNELS})

set(FLIGHT_MODES
    MANUAL STABILIZE
    FLY_BY_WIRE_A
    FLY_BY_WIRE_B
    RTL
    AUTO
    LOITER CIRCLE)

set(FLIGHT_MODES_RADIO_PWM      1165 1295 1425 1555 1685 1815)

set(FLIGHT_MODES_DEFAULT
    RTL
    RTL
    STABILIZE
    STABILIZE
    MANUAL
    MANUAL)

foreach(MODE 1 2 3 4 5 6)
    math(EXPR INDEX "${MODE} -1")
    list(GET FLIGHT_MODES_RADIO_PWM ${INDEX} RADIO_PWM)
    list(GET FLIGHT_MODES_DEFAULT ${INDEX} FLIGHT_MODE_DEFAULT)
    apm_option("FLIGHT_MODE_${MODE}" TYPE STRING ADVANCED
        DESCRIPTION "Flight mode ${MODE} (pwm ${RADIO_PWM} ms)?"
        DEFAULT "${FLIGHT_MODE_DEFAULT}"
        OPTIONS ${FLIGHT_MODES})
endforeach()

#set(FLAP_1_SPEED "0" CACHE STRING "Speed below which flaps are deployed (m/s)?")
#set(FLAP_1_PERCENT "0" CACHE STRING "Flap deployment percentage (%)?")

#set(FLAP_2_SPEED "0" CACHE STRING "Speed below which flaps are deployed (m/s)?")
#set(FLAP_2_PERCENT "0" CACHE STRING "Flap deployment percentage (%)?")

#set(THROTTLE_FAILSAFE "ENABLED" CACHE STRING "Enable throttle shuttoff when radio below failsafe value?")
#set_property(CACHE THROTTLE_FAILSAFE PROPERTY STRINGS ENABLED DISABLED) 

#set(THROTTLE_FS_VALUE "950" CACHE STRING "Radio value at which to disable throttle (ms).")

#set(GCS_HEARTBEAT_FAILSAFE "DISABLED" CACHE STRING "Enable failsafe when ground station communication lost?")
#set_property(CACHE GCS_HEARTBEAT_FAILSAFE PROPERTY STRINGS ENABLED DISABLED) 
#set(SHORT_FAILSAFE_ACTION "0" CACHE STRING "Failsafe mode RTL, then return to AUTO/LOITER")
#set(LONG_FAILSAFE_ACTION "0" CACHE STRING "Failsafe mode RTL")

#set(AUTO_TRIM "DISABLED" CACHE STRING "Update trim with manual radio input when leaving MANUAL mode?")
#set_property(CACHE AUTO_TRIM PROPERTY STRINGS ENABLED DISABLED) 

#set(THROTTLE_REVERSE "DISABLED" CACHE STRING "Reverse throttle output signal?")
#set_property(CACHE THROTTLE_REVERSE PROPERTY STRINGS ENABLED DISABLED) 

#set(ENABLE_STICK_MIXING "DISABLED" CACHE STRING "Enable manual input in autopilot modes?")
#set_property(CACHE THROTTLE_REVERSE PROPERTY STRINGS ENABLED DISABLED) 

#set(THROTTLE_OUT "ENABLED" CACHE STRING "Disabled throttle output? (useful for debugging)?")
#set_property(CACHE THROTTLE_OUT PROPERTY STRINGS ENABLED DISABLED) 

#set(GROUND_START_DELAY "0" CACHE STRING "Delay between power-up and INS calibration (s)?")

#set(ENABLE_AIR_START "DISABLED" CACHE STRING "Enable in-air restart?")
#set_property(CACHE ENABLE_AIR_START PROPERTY STRINGS ENABLED DISABLED) 


