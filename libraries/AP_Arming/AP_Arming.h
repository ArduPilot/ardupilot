/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_ARMING_H__
#define __AP_ARMING_H__ 

#include <AP_AHRS.h>
#include <AP_HAL.h>
#include <GCS_MAVLink.h>

class AP_Arming {
public:
    enum ArmingChecks {
        ARMING_CHECK_NONE       = 0x0000,
        ARMING_CHECK_ALL        = 0x0001,
        ARMING_CHECK_BARO       = 0x0002,
        ARMING_CHECK_COMPASS    = 0x0004,
        ARMING_CHECK_GPS        = 0x0008,
        ARMING_CHECK_INS        = 0x0010,
        ARMING_CHECK_RC         = 0x0020,
        ARMING_CHECK_VOLTAGE    = 0x0040,
        ARMING_CHECK_BATTERY    = 0x0080,

        ARMING_CHECK_UNINITILIZED = 0x8000
    };

    enum ArmingMethod {
        NONE = 0,
        RUDDER,
        MAVLINK
    };

    //for the hacky funciton pointer to gcs_send_text_p
    typedef void (*gcs_send_t_p)(gcs_severity, const prog_char_t*);

protected:
    bool                                                armed;

    //bitmask for which checks are required
    uint16_t                            arming_checks_enabled;

    //how the vehicle was armed
    uint8_t                                     arming_method;

    const AP_AHRS                                       &ahrs;
    const AP_Baro                                  &barometer;
    const AP_HAL::HAL                                    &hal;
    const bool                                   &home_is_set;
    gcs_send_t_p                              gcs_send_text_P;

public:
    AP_Arming(const AP_AHRS &ahrs_ref, const AP_Baro &baro,
            const AP_HAL::HAL &hal_ref,const bool &home_set, gcs_send_t_p);

    bool is_armed();

    bool enabled_checks_make_sense();

    uint16_t get_enabled_checks();

    void set_enabled_checks(uint16_t);

    bool barometer_checks();

    bool compass_checks();

    bool gps_checks();

    bool battery_checks();

    bool hardware_safety_check();

    bool manual_transmitter_checks();

    bool pre_arm_checks();

    bool arm(uint8_t method);

    bool disarm();

    uint8_t get_arming_method();

};

#endif //__AP_ARMING_H__
