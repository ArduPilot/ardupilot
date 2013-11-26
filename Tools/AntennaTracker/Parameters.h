// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <AP_Common.h>

// Global parameter class.
//
class Parameters {
public:

    /*
     *  The value of k_format_version determines whether the existing
     *  eeprom data is considered valid. You should only change this
     *  value under the following circumstances:
     *
     *  1) the meaning of an existing eeprom parameter changes
     *
     *  2) the value of an existing k_param_* enum value changes
     *
     *  Adding a new parameter should _not_ require a change to
     *  k_format_version except under special circumstances. If you
     *  change it anyway then all ArduPlane users will need to reload all
     *  their parameters. We want that to be an extremely rare
     *  thing. Please do not just change it "just in case".
     *
     *  To determine if a k_param_* value has changed, use the rules of
     *  C++ enums to work out the value of the neighboring enum
     *  values. If you don't know the C++ enum rules then please ask for
     *  help.
     */

    //////////////////////////////////////////////////////////////////
    // STOP!!! DO NOT CHANGE THIS VALUE UNTIL YOU FULLY UNDERSTAND THE
    // COMMENTS ABOVE. IF UNSURE, ASK ANOTHER DEVELOPER!!!
    static const uint16_t k_format_version = 1;
    //////////////////////////////////////////////////////////////////


    // The parameter software_type is set up solely for ground station use
    // and identifies the software type (eg ArduPilotMega versus ArduCopterMega)
    // GCS will interpret values 0-9 as ArduPilotMega.  Developers may use
    // values within that range to identify different branches.
    //
    static const uint16_t k_software_type = 4;

    enum {
        // Layout version number, always key zero.
        //
        k_param_format_version = 0,
        k_param_software_type,

        k_param_gcs0 = 100,         // stream rates for port0
        k_param_gcs3,               // stream rates for port3
        k_param_sysid_this_mav,
        k_param_sysid_my_gcs,
        k_param_serial0_baud,
        k_param_serial3_baud,
        k_param_imu,
        k_param_compass_enabled,
        k_param_compass,
        k_param_ahrs,  // AHRS group
        k_param_barometer,
        k_param_scheduler,
        k_param_ins,
        k_param_sitl,
        k_param_pidPitch2Srv,
        k_param_pidYaw2Srv,

        k_param_channel_yaw = 200,
        k_param_channel_pitch

        // 254,255: reserved
    };

    AP_Int16 format_version;
    AP_Int8 software_type;

    // Telemetry control
    //
    AP_Int16 sysid_this_mav;
    AP_Int16 sysid_my_gcs;
    AP_Int8 serial0_baud;
    AP_Int8 serial3_baud;

    AP_Int8 compass_enabled;

    // PID controllers
    PID         pidPitch2Srv;
    PID         pidYaw2Srv;

    Parameters() :
        pidPitch2Srv(1.0f, 0.2f, 0.05f, 4000.0f),
        pidYaw2Srv(1.0f, 0.2f, 0.05f, 4000.0f)
        {}
};

extern const AP_Param::Info var_info[];

#endif // PARAMETERS_H
