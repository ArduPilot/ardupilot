// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file    AP_Land.h
/// @brief    Land control library

#ifndef AP_LAND_H
#define AP_LAND_H

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

/// @class    AP_Land
/// @brief    Class managing the control of landing
class AP_Land {

public:

    /// Constructor
    AP_Land();
//    {
//        // setup parameter defaults
//        AP_Param::setup_object_defaults(this, var_info);
//    };

    static const struct AP_Param::GroupInfo        var_info[];
    
    AP_Int16 pitch_cd;
    AP_Float flare_sec;
    AP_Float flare_alt;
    AP_Float slope;
    AP_Int16 flare_thr;
    AP_Int16 rev_pt_dn;
    AP_Int16 rev_pt_up;   
    AP_Int16 pre_flare_thr;
    AP_Int16 pre_flare_alt;
    AP_Int16 app_spd_const;
    AP_Float app_tar_spd;
    AP_Int8 disarm_delay;
    AP_Int8 abort_throttle_enable;
    AP_Int8 flap_percent;
    AP_Int8 rev_on_gnd;
    AP_Int16 thr_min_rev_pwm;
	AP_Int16 bat_offset;
};

#endif /* AP_LAND_H */
