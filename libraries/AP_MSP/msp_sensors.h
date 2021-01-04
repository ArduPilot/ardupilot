#include <stdint.h>

namespace MSP
{
// src/main/msp/msp_protocol_v2_sensor_msg.h
typedef struct PACKED {
    uint8_t quality;    // [0;255]
    int32_t distance_mm; // Negative value for out of range
} msp_rangefinder_data_message_t;

typedef struct PACKED {
    uint8_t quality;    // [0;255]
    int32_t motion_x;
    int32_t motion_y;
} msp_opflow_data_message_t;

typedef struct PACKED {
    uint8_t  instance;                  // sensor instance number to support multi-sensor setups
    uint16_t gps_week;                   // GPS week, 0xFFFF if not available
    uint32_t ms_tow;
    uint8_t  fix_type;
    uint8_t  satellites_in_view;
    uint16_t horizontal_pos_accuracy;     // [cm]
    uint16_t vertical_pos_accuracy;       // [cm]
    uint16_t horizontal_vel_accuracy;     // [cm/s]
    uint16_t hdop;
    int32_t  longitude;
    int32_t  latitude;
    int32_t  msl_altitude;       // cm
    int32_t  ned_vel_north;       // cm/s
    int32_t  ned_vel_east;
    int32_t  ned_vel_down;
    uint16_t ground_course;      // deg * 100, 0..36000
    uint16_t true_yaw;           // deg * 100, values of 0..36000 are valid. 65535 = no data available
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  min;
    uint8_t  sec;
} msp_gps_data_message_t;

typedef struct PACKED {
    uint8_t instance;
    uint32_t time_ms;
    float pressure_pa;
    int16_t temp; // centi-degrees C
} msp_baro_data_message_t;

typedef struct PACKED {
    uint8_t instance;
    uint32_t time_ms;
    int16_t magX; // mGauss, front
    int16_t magY; // mGauss, right
    int16_t magZ; // mGauss, down
} msp_compass_data_message_t;

typedef struct PACKED {
    uint8_t instance;
    uint32_t time_ms;
    float pressure;
    int16_t temp; // centi-degrees C
} msp_airspeed_data_message_t;
}
