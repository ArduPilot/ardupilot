// Class supporting autorotation state within the heli rotor speed controller

#pragma once

#include <AP_Param/AP_Param.h>

// helper class to manage autorotation state and variables within RSC
class RSC_Autorotation
{
public:

    RSC_Autorotation(void);

    enum class State {
        DEACTIVATED,
        BAILING_OUT,
        ACTIVE,
    };

    // state accessors
    bool active(void) const { return state == State::ACTIVE; }
    bool bailing_out(void) const { return state == State::BAILING_OUT; }
    bool enabled(void) const { return enable.get() == 1; }

    // update idle throttle when in autorotation
    bool get_idle_throttle(float& idle_throttle);

    // get the throttle ramp rate needed when bailing out of autorotation
    float get_bailout_ramp(void) const;

    // get the allowed run-up time that we expect the rotor to need to complete a bailout
    float get_runup_time(void) const;

    // request changes in autorotation state
    void set_active(bool active, bool force_state);

    // sanity check of parameters, should be called only whilst disarmed
    bool arming_checks(size_t buflen, char *buffer) const;

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

private:

    AP_Int8  idle_output;             // (percent) rsc output used when in autorotation, used for setting autorotation window on ESCs
    AP_Int8  bailout_throttle_time;   // (seconds) time for in-flight power re-engagement when bailing-out of an autorotation
    AP_Int8  bailout_runup_time;      // (seconds) expected time for the motor to fully engage and for the rotor to regain safe head speed if necessary
    AP_Int8  enable;                  // enables autorotation state within the RSC

    State state;
    uint32_t bail_out_started_ms;     // (milliseconds) time that bailout started, used to time transition from "bailing out" to "autorotation stopped"

};
