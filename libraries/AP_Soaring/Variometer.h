
/* Variometer class by Samuel Tabor

Manages the estimation of aircraft total energy, drag and vertical air velocity.
*/
#pragma once

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Param/AP_Param.h>
#include <DataFlash/DataFlash.h>
#include <AP_SpdHgtControl/AP_SpdHgtControl.h>

#define ASPD_FILT 0.05
#define TE_FILT 0.03
#define TE_FILT_DISPLAYED 0.15

class Variometer {

    AP_AHRS &_ahrs;
    const AP_Vehicle::FixedWing &_aparm;

    // store time of last update
    unsigned long _prev_update_time;

    float _last_alt;

    float _aspd_filt;
    float _last_aspd;
    float _last_roll;
    float _last_total_E;

public:
    Variometer(AP_AHRS &ahrs, const AP_Vehicle::FixedWing &parms);
    float alt;
    float reading;
    float filtered_reading;
    float displayed_reading;
    bool new_data;

    void update(const float polar_K, const float polar_CD0, const float polar_B);
    float correct_netto_rate(float climb_rate, float phi, float aspd, const float polar_K, const float polar_CD0, const float polar_B);

};

