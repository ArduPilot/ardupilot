# options
#bool_option(LOGGING DESCRIPTION "Logging support?" DEFAULT OFF)
#bool_option(GPS "Gps support?" ON)
#option(CLI_SLIDER "Command-line-interface slider support?" OFF)
#option(APM2 "Build for APM 2.0" OFF)

apm_option("APM_FRAME" TYPE "COMBO"
    DESCRIPTION "Vehicle type?"
    DEFAULT "PLANE_FRAME"
    OPTIONS "HELI_FRAME" "HEXA_FRAME" "OCTA_FRAME" "Y6_FRAME")

apm_option("GPS_PROTOCOL" TYPE "COMBO"
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

apm_option("AIRSPEED_SENSOR" TYPE "BOOL"
    DESCRIPTION "Enable airspeed sensor?"
    DEFAULT OFF)

apm_option("AIRSPEED_RATIO" TYPE "STRING" ADVANCED
    DESCRIPTION "Airspeed ratio?"
    DEFAULT "1.9936")

apm_option("MAGNETOMETER" TYPE "BOOL"
    DESCRIPTION "Enable airspeed sensor?"
    DEFAULT OFF)

apm_option("MAG_ORIENTATION" TYPE "COMBO" ADVANCED
    DESCRIPTION "Magnetometer orientation?" 
    DEFAULT "AP_COMPASS_COMPONENTS_DOWN_PINS_FORWARD"
    OPTIONS 
        "AP_COMPASS_COMPONENTS_DOWN_PINS_FORWARD"
        "AP_COMPASS_COMPONENTS_DOWN_PINS_BACK"
        "AP_COMPASS_COMPONENTS_UP_PINS_FORWARD"
        "AP_COMPASS_COMPONENTS_UP_PINS_BACK")

apm_option("HIL_MODE" TYPE "COMBO"
    DESCRIPTION "Hardware-in-the-loop- mode?"
    DEFAULT "HIL_MODE_DISABLED"
    OPTIONS 
        "HIL_MODE_DISABLED"
        "HIL_MODE_ATTITUDE"
        "HIL_MODE_SENSORS")

apm_option("HIL_PORT" TYPE "COMBO"
    DESCRIPTION "Port for Hardware-in-the-loop communication"
    DEFAULT "0"
    OPTIONS "0" "1" "2" "3")

apm_option("HIL_PROTOCOL" TYPE "COMBO"
    DESCRIPTION "Hardware-in-the-loop protocol?"
    DEFAULT "HIL_PROTOCOL_MAVLINK"
    OPTIONS "HIL_PROTOCOL_MAVLINK" "HIL_PROTOCOL_XPLANE")

apm_option("GPS_PROTOCOL" TYPE "COMBO"
    DESCRIPTION "Ground station protocol?"
    DEFAULT "GCS_PROTOCOL_MAVLINK"
    OPTIONS "GCS_PROTOCOL_NONE" "GCS_PROTOCOL_MAVLINK")

apm_option("GCS_PORT" TYPE "COMBO" ADVANCED
    DESCRIPTION "Ground station port?"
    DESCRIPTION "3"
    OPTIONS "0" "1" "2" "3")

apm_option("MAV_SYSTEM_ID" TYPE "STRING" ADVANCED
    DESCRIPTION "MAVLink System ID?"
    DESCRIPTION "1")

apm_option("SERIAL0_BAUD" TYPE "COMBO" ADVANCED
    DESCRIPTION "Serial 0 baudrate?"
    DEFAULT "115200" 
    OPTIONS "57600" "115200") 

apm_option("SERIAL3_BAUD" TYPE "COMBO" ADVANCED
    DESCRIPTION "Serial 3 baudrate?"
    DEFAULT "57600" 
    OPTIONS "57600" "115200") 

apm_option("BATTERY_EVENT" TYPE "BOOL" ADVANCED
    DESCRIPTION "Enable low voltage/ high discharge warnings?"
    DEFAULT OFF)

apm_option("LOW_VOLTAGE" TYPE "STRING" ADVANCED
    DESCRIPTION "Voltage to consider low (volts)?"
    DEFAULT "9.6")

apm_option("VOLT_DIV_RATIO" TYPE "STRING" ADVANCED
    DESCRIPTION "Voltage division ratio?"
    DEFAULT "3.56")

apm_option("CUR_AMPS_PER_VOLT" TYPE "STRING" ADVANCED
    DESCRIPTION "Current amps/volt?"
    DEFAULT "27.32")

apm_option("CUR_AMPS_PER_VOLT" TYPE "STRING" ADVANCED
    DESCRIPTION "Current amps/volt?"
    DEFAULT "27.32")

#set(CUR_AMPS_PER_VOLT "27.32" CACHE STRING "Current amps/volt?")
#set(CUR_AMPS_OFFSET "0.0" CACHE STRING "Current amps offset?")
#set(HIGH_DISCHARGE "1760" CACHE STRING "What to consider high discharge rate (milliamp-hours)?")

#set(INPUT_VOLTAGE "4.68" CACHE STRING "Voltage measured at the processor (volts)?")

#set(FLIGHT_MODE_CHANNEL "8" CACHE STRING "The radio channel to control the flight mode.")
#set_property(CACHE FLIGHT_MODE_CHANNEL PROPERTY STRINGS 1 2 3 4 5 6 7 8)

#set(FLIGHT_MODES MANUAL STABILIZE FLY_BY_WIRE_A FLY_BY_WIRE_B RTL AUTO LOITER CIRCLE)

#set(FLIGHT_MODE_1 "RTL" CACHE STRING "Flight mode for radio position 1 (1165 ms)?")
#set_property(CACHE FLIGHT_MODE_1 PROPERTY STRINGS ${FLIGHT_MODES}) 

#set(FLIGHT_MODE_2 "RTL" CACHE STRING "Flight mode for radio position 2 (1295 ms)?")
#set_property(CACHE FLIGHT_MODE_2 PROPERTY STRINGS ${FLIGHT_MODES}) 

#set(FLIGHT_MODE_3 "STABILIZE" CACHE STRING "Flight mode for radio position 3 (1425 ms)?")
#set_property(CACHE FLIGHT_MODE_3 PROPERTY STRINGS ${FLIGHT_MODES}) 

#set(FLIGHT_MODE_4 "STABILIZE" CACHE STRING "Flight mode for radio position 4 (1555 ms)?")
#set_property(CACHE FLIGHT_MODE_4 PROPERTY STRINGS ${FLIGHT_MODES}) 

#set(FLIGHT_MODE_5 "MANUAL" CACHE STRING "Flight mode for radio position 5 (FAILSAFE if using channel 8) (1685 ms)?")
#set_property(CACHE FLIGHT_MODE_5 PROPERTY STRINGS ${FLIGHT_MODES}) 

#set(FLIGHT_MODE_6 "MANUAL" CACHE STRING "Flight mode for radio position 6 (FAILSAFE is using channel 8) (1815 ms)?")
#set_property(CACHE FLIGHT_MODE_6 PROPERTY STRINGS ${FLIGHT_MODES}) 

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

#set(GROUND_START_DELAY "0" CACHE STRING "Delay between power-up and IMU calibration (s)?")

#set(ENABLE_AIR_START "DISABLED" CACHE STRING "Enable in-air restart?")
#set_property(CACHE ENABLE_AIR_START PROPERTY STRINGS ENABLED DISABLED) 


