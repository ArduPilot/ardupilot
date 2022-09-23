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

    //bool is_available_gps_latlng(void) { return true; }
    //bool is_available_gps_status(void) { return true; }
    //bool is_available_attiandrng(void) { return true; }
    //bool is_available_velandyaw(void) { return true; }
    bool is_available_batt(uint8_t instance);
    bool is_available_ap_status(void);
    //bool is_available_home(void) { return true; }
    bool is_available_rpm(void);
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
    uint16_t prep_number(int32_t number, uint8_t digits, uint8_t power);

    static AP_Frsky_SPort_Protocol *singleton;
};

namespace AP {
    AP_Frsky_SPort_Protocol *frsky_sport_protocol();
};
