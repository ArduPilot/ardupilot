#ifndef UTIL_H_
#define UTIL_H_

struct Location {
    int32_t alt;
    int32_t lat;
    int32_t lng;
    uint8_t options;
};

typedef char prog_char_t;

enum ap_message {
    MSG_HEARTBEAT,
    MSG_ATTITUDE,
    MSG_LOCATION,
    MSG_EXTENDED_STATUS1,
    MSG_EXTENDED_STATUS2,
    MSG_NAV_CONTROLLER_OUTPUT,
    MSG_CURRENT_WAYPOINT,
    MSG_VFR_HUD,
    MSG_RADIO_OUT,
    MSG_RADIO_IN,
    MSG_RAW_IMU1,
    MSG_RAW_IMU2,
    MSG_RAW_IMU3,
    MSG_GPS_RAW,
    MSG_SYSTEM_TIME,
    MSG_SERVO_OUT,
    MSG_NEXT_WAYPOINT,
    MSG_NEXT_PARAM,
    MSG_STATUSTEXT,
    MSG_LIMITS_STATUS,
    MSG_FENCE_STATUS,
    MSG_AHRS,
    MSG_SIMSTATE,
    MSG_HWSTATUS,
    MSG_WIND,
    MSG_RANGEFINDER,
    MSG_RETRY_DEFERRED // this must be last
};

enum gcs_severity {
    SEVERITY_LOW=1,
    SEVERITY_MEDIUM,
    SEVERITY_HIGH,
    SEVERITY_CRITICAL,
    SEVERITY_USER_RESPONSE
};

struct RallyLocation {
    int32_t  lat;        //Latitude * 10^7
    int32_t  lng;        //Longitude * 10^7
    int16_t  alt;        //transit altitude (and loiter altitude) in meters;
    int16_t  break_alt;  //when autolanding, break out of loiter at this alt (meters)
    uint16_t land_dir;   //when the time comes to auto-land, try to land in this direction (centidegrees)
    uint8_t  flags;      //bit 0 = seek favorable winds when choosing a landing poi
                         //bit 1 = do auto land after arriving
                         //all other bits are for future use.
};

#endif /* UTIL_H_ */
