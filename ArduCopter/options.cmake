# options
apm_option("APM_PROGRAMMING_PORT" TYPE STRING
    DESCRIPTION "Programming upload port?"
    DEFAULT "/dev/ttyUSB0")

apm_option("APM_BOARD" TYPE STRING
    DESCRIPTION "ArduPilotMega board?" 
    DEFAULT "mega2560"
    OPTIONS "mega" "mega2560")

apm_option("APM_FRAME" TYPE STRING
    DESCRIPTION "Vehicle type?"
    DEFAULT "PLANE_FRAME"
    OPTIONS
        "PLANE FRAME"
        "HELI_FRAME"
        "HEXA_FRAME"
        "OCTA_FRAME"
        "Y6_FRAME"
    )

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

apm_option("AIRSPEED_RATIO" TYPE STRING ADVANCED
    DESCRIPTION "Airspeed ratio?"
    DEFAULT "1.9936")

apm_option("MAGNETOMETER" TYPE BOOL
    DESCRIPTION "Enable airspeed sensor?"
    DEFAULT OFF)

apm_option("MAG_ORIENTATION" TYPE STRING ADVANCED
    DESCRIPTION "Magnetometer orientation?" 
    DEFAULT "AP_COMPASS_COMPONENTS_DOWN_PINS_FORWARD"
    OPTIONS 
        "AP_COMPASS_COMPONENTS_DOWN_PINS_FORWARD"
        "AP_COMPASS_COMPONENTS_DOWN_PINS_BACK"
        "AP_COMPASS_COMPONENTS_UP_PINS_FORWARD"
        "AP_COMPASS_COMPONENTS_UP_PINS_BACK")

apm_option("HIL_MODE" TYPE STRING
    DESCRIPTION "Hardware-in-the-loop- mode?"
    DEFAULT "HIL_MODE_DISABLED"
    OPTIONS 
        "HIL_MODE_DISABLED"
        "HIL_MODE_ATTITUDE"
        "HIL_MODE_SENSORS")

apm_option("HIL_PORT" TYPE STRING
    DESCRIPTION "Port for Hardware-in-the-loop communication"
    DEFAULT "0"
    OPTIONS "0" "1" "2" "3")

apm_option("HIL_PROTOCOL" TYPE STRING
    DESCRIPTION "Hardware-in-the-loop protocol?"
    DEFAULT "HIL_PROTOCOL_MAVLINK"
    OPTIONS "HIL_PROTOCOL_MAVLINK" "HIL_PROTOCOL_XPLANE")

apm_option("GPS_PROTOCOL" TYPE STRING
    DESCRIPTION "Ground station protocol?"
    DEFAULT "GCS_PROTOCOL_MAVLINK"
    OPTIONS "GCS_PROTOCOL_NONE" "GCS_PROTOCOL_MAVLINK")

apm_option("GCS_PORT" TYPE STRING ADVANCED
    DESCRIPTION "Ground station port?"
    DESCRIPTION "3"
    OPTIONS "0" "1" "2" "3")

apm_option("MAV_SYSTEM_ID" TYPE STRING ADVANCED
    DESCRIPTION "MAVLink System ID?"
    DESCRIPTION "1")

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

apm_option("CUR_AMPS_OFFSET" TYPE STRING ADVANCED
    DESCRIPTION "Current amps offset?"
    DEFAULT "0.0")
