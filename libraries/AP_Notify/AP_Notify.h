/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_NOTIFY_H__
#define __AP_NOTIFY_H__

#include <AP_Common.h>

class AP_Notify
{
public:

    /// definition of callback function
    typedef void (*update_fn_t)(void);

    /// notify_type - bitmask of notification types
    struct notify_type {
        uint16_t initialising   : 1;    // 1 if initialising and copter should not be moved
        uint16_t gps_status     : 2;    // 0 = no gps, 1 = no lock, 2 = 2d lock, 3 = 3d lock
        uint16_t armed          : 1;    // 0 = disarmed, 1 = armed
        uint16_t pre_arm_check  : 1;    // 0 = failing checks, 1 = passed
        uint16_t save_trim      : 1;    // 1 if gathering trim data
        uint16_t esc_calibration: 1;    // 1 if calibrating escs
    };

    static struct notify_type flags;

    /// register_callback - allows registering functions to be called with AP_Notify::update() is called from main loop
    static void register_update_function(update_fn_t fn) {_update_fn = fn;}

    /// update - allow updates of leds that cannot be updated during a timed interrupt
    static void update() { if (AP_Notify::_update_fn != NULL) AP_Notify::_update_fn(); }

    /// To-Do: potential notifications to add

    /// flight_mode 
    /// void flight_mode(uint8_t mode) = 0;

    /// throttle failsafe
    /// void fs_throttle(bool uint8_t);     // 0 if throttle failsafe is cleared, 1 if activated
    /// void fs_battery(bool uint8_t);      // 1 if battery voltage is low or consumed amps close to battery capacity, 0 if cleared
    /// void fs_gps(bool uint8_t);          // 1 if we've lost gps lock and it is required for our current flightmode, 0 if cleared
    /// void fs_gcs(bool uint8_t);          // 1 if we've lost contact with the gcs and it is required for our current flightmode or pilot input method, 0 if cleared
    /// void fence_breach(bool uint8_t);    // fence type breached or 0 if cleared

    /// switch_aux1(uint8_t state);     // 0 if aux switch is off, 1 if in middle, 2 if high
    /// switch_aux2(uint8_t state);     // 0 if aux switch is off, 1 if in middle, 2 if high

    /// reached_waypoint();             // called after we reach a waypoint

    /// error(uint8_t subsystem_id, uint8_t error_code);    // general error reporting

    /// objects that we expect to create:
    /// apm2, px4 leds
    /// copter leds
    /// blinkm
    /// buzzer
//private:
    static update_fn_t _update_fn;

};

#endif	// __AP_NOTIFY_H__
