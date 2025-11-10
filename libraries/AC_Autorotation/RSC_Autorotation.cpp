#include "RSC_Autorotation.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/AP_HAL.h>

#define RSC_AROT_RAMP_TIME_DEFAULT 2 // time in seconds to ramp motors when bailing out of autorotation

extern const AP_HAL::HAL& hal;

// RSC autorotation state specific parameters
const AP_Param::GroupInfo RSC_Autorotation::var_info[] = {

    // @Param: ENBL
    // @DisplayName: Enable autorotation handling in RSC
    // @Description: Allows you to enable (1) or disable (0) the autorotation functionality within the Rotor Speed Controller.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENBL", 1, RSC_Autorotation, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: RAMP
    // @DisplayName: Time for in-flight power re-engagement when exiting autorotations
    // @Description: When exiting an autorotation in a bailout manoeuvre, this is the time in seconds for the throttle output (HeliRSC servo) to ramp from idle (H_RSC_AROT_IDLE) to flight throttle setting when motor interlock is re-enabled. When using an ESC with an autorotation bailout function, this parameter should be set to 0.1 (minimum value).
    // @Range: 0.1 10
    // @Units: s
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("RAMP", 2, RSC_Autorotation, bailout_throttle_time, RSC_AROT_RAMP_TIME_DEFAULT),

    // @Param: IDLE
    // @DisplayName: Idle throttle percentage during autorotation
    // @Description: Idle throttle used for during autotoration. For external governors, this would be set to a value that is within the autorotation window of the governer/ESC to enable fast spool-up, when bailing out of an autorotation.  Set 0 to disable.
    // @Range: 0 40
    // @Units: %
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("IDLE", 3, RSC_Autorotation, idle_output, 0.0),

    // @Param: RUNUP
    // @DisplayName: Time allowed for in-flight power re-engagement
    // @Description: When exiting an autorotation in a bailout manoeuvre, this is the expected time in seconds for the main rotor to reach full speed after motor interlock is enabled. Must be at least one second longer than the H_RSC_AROT_RAMP time that is set. This timer should be set for at least the amount of time it takes to get your helicopter to full flight power. Failure to heed this warning could result in early entry into autonomously controlled collective modes (e.g. alt hold, loiter, etc), whereby the collective could be raised before the engine has reached full power, with a subsequently dangerous slowing of head speed.
    // @Range: 1 10
    // @Units: s
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("RUNUP", 4, RSC_Autorotation, bailout_runup_time, RSC_AROT_RAMP_TIME_DEFAULT+1),

    AP_GROUPEND
};

RSC_Autorotation::RSC_Autorotation(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// set the desired autorotation state
// this state machine handles the transition from active to deactivated via the bailout logic
// to force the state to be immediately deactivated, then the force_state bool is used
void RSC_Autorotation::set_active(bool active, bool force_state)
{
    if (!enabled()) {
        return;
    }

    // set the desired state based on the bool. We only set either ACTIVE or DEACTIVATED
    // here and let the autorotation state machine and RSC runup code handle the bail out case
    RSC_Autorotation::State desired_state = active ? RSC_Autorotation::State::ACTIVE : RSC_Autorotation::State::DEACTIVATED;

    // don't do anything if desired state is already set
    if (desired_state == state) {
        return;
    }

    // Handle the transition from the ACTIVE to DEACTIVATED states via the BAILING_OUT case
    // set the bailout case if deactivated has just been requested
    if ((state == State::ACTIVE) && (desired_state == State::DEACTIVATED) && !force_state) {
        desired_state = State::BAILING_OUT;
        bail_out_started_ms = AP_HAL::millis();
    }

    // Wait for allocated autorotation run up time before we allow progression of state to deactivated
    if ((state == State::BAILING_OUT) && 
        (desired_state == State::DEACTIVATED) &&
        (bail_out_started_ms > 0) && 
        (AP_HAL::millis() - bail_out_started_ms < uint32_t(get_runup_time()*1000)))
    {
        return;
    }

    // handle GCS messages
    switch (desired_state)
    {
    case State::DEACTIVATED:
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "RSC: Autorotation Stopped");
        break;

    case State::BAILING_OUT:
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "RSC: Bailing Out");
        break;

    case State::ACTIVE:
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "RSC: In Autorotation");
        break;

    default:
        // do nothing
        break;
    }

    // Actually set the state
    state = desired_state;
}

bool RSC_Autorotation::get_idle_throttle(float& idle_throttle)
{
    if (state != State::ACTIVE) {
        // We do not want to use autorotation idle throttle
        return false;
    }

    if (idle_output.get() <= 0) {
        // If autorotation idle is not set, do not modify idle throttle as we just use H_RSC_IDLE
        // Heli with an ICE engine is an example of this type of config
        return true;
    }

    // if we are autorotating and the autorotation idle throttle param is set we want to
    // to output this as the idle throttle for ESCs with an autorotation window
    idle_throttle = constrain_float(idle_output.get()*0.01, 0.0, 0.4);

    return true;
}

float RSC_Autorotation::get_bailout_ramp(void) const
{
    // Allow ramp times as quick as 0.1 of a second for ESCs with autorotation windows
    return MAX(float(bailout_throttle_time.get()), 0.1);
}

float RSC_Autorotation::get_runup_time(void) const
{
    // If we are in the autorotation state we want the rotor speed model to ramp down rapidly to zero, ensuring we get past
    // the critical rotor speed, and therefore triggering a proper bailout should we re-engage the interlock at any point
    if (state == State::ACTIVE) {
        return 0.1;
    }

    // Never let the runup timer be less than the throttle ramp time
    return (float) MAX(bailout_throttle_time.get() + 1, bailout_runup_time.get());
}

// sanity check of parameters, should be called only whilst disarmed
bool RSC_Autorotation::arming_checks(size_t buflen, char *buffer) const
{
    // throttle runup must be larger than ramp, keep the params up to date to not confuse users
    if (bailout_throttle_time.get() + 1 > bailout_runup_time.get()) {
        hal.util->snprintf(buffer, buflen, "H_RSC_AROT_RUNUP must be > H_RSC_AROT_RAMP");
        return false;
    }
    return true;
}
