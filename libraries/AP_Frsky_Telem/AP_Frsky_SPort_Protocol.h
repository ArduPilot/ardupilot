#pragma once

#include <AP_HAL/AP_HAL.h>


class AP_Frsky_SPort_Protocol
{
public:

    AP_Frsky_SPort_Protocol()
    {
        singleton = this;
    }

    static AP_Frsky_SPort_Protocol *get_singleton(void)
    {
        if (!singleton) { // && !hal.util->get_soft_armed()) {
            new AP_Frsky_SPort_Protocol();
        }
        return singleton;
    }

    enum PassthroughParam : uint8_t {
        NONE =                0,
        FRAME_TYPE =          1,
        BATT_FS_VOLTAGE =     2,
        BATT_FS_CAPACITY =    3,
        BATT_CAPACITY_1 =     4,
        BATT_CAPACITY_2 =     5,
        TELEMETRY_FEATURES =  6
    };

    enum PassthroughFeatures : uint8_t {
        BIDIR =               0,
        SCRIPTING =           1,
    };

    enum {
        GPS_LONG_LATI_FIRST_ID = 0x0800,
        DIY_FIRST_ID =           0x5000,
    };

    enum {
        FRSKY_ID_GPS_LAT_LON =        0x0800,
        FRSKY_ID_TEXT =               0x5000,
        FRSKY_ID_AP_STATUS =          0x5001,
        FRSKY_ID_GPS_STATUS =         0x5002,
        FRSKY_ID_BATT_1 =             0x5003,
        FRSKY_ID_HOME =               0x5004,
        FRSKY_ID_VEL_YAW =            0x5005,
        FRSKY_ID_ATTITUDE_RANGE =     0x5006,
        FRSKY_ID_PARAM =              0x5007,
        FRSKY_ID_BATT_2 =             0x5008,
        FRSKY_ID_RPM =                0x500A,
        FRSKY_ID_TERRAIN =            0x500B,
        FRSKY_ID_WIND =               0x500C,
        FRSKY_ID_WAYPOINT_V2 =        0x500D,
    };

    //bool is_available_gps_latlng(void) { return true; }
    //bool is_available_gps_status(void) { return true; }
    //bool is_available_attiandrng(void) { return true; }
    //bool is_available_velandyaw(void) { return true; }
    bool is_available_batt(uint8_t instance);
    bool is_available_batt(void) { return is_available_batt(0); }
    bool is_available_ap_status(void);
    //bool is_available_home(void) { return true; }
    bool is_available_rpm(uint8_t instance);
    bool is_available_rpm(void) { return is_available_rpm(0); }
    bool is_available_terrain(void);
    bool is_available_wind(void);
    bool is_available_waypoint(void);

    uint32_t calc_gps_latlng(bool &send_latitude);
    uint32_t calc_gps_status(void);
    uint32_t calc_attiandrng(void);
    uint32_t calc_velandyaw(bool airspeed_enabled, bool send_airspeed);
    uint32_t calc_batt(uint8_t instance);
    uint32_t calc_ap_status(void);
    uint32_t calc_home(void);
    uint32_t calc_rpm(void);
    uint32_t calc_terrain(void);
    uint32_t calc_wind(void);
    uint32_t calc_waypoint(void);

    uint32_t calc_param(uint8_t* param_id);

    void pack_packet(uint8_t* buf, uint8_t count, uint16_t id, uint32_t data);

    float get_vspeed_ms(void);
    float get_current_height_cm(void);

    uint16_t prep_number(int32_t number, uint8_t digits, uint8_t power);

    int32_t calc_sensor_rpm(uint8_t instance);

    static AP_Frsky_SPort_Protocol *singleton;
};

namespace AP {
    AP_Frsky_SPort_Protocol *frsky_sport_protocol();
};
