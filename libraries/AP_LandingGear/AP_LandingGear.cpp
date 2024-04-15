#include "AP_LandingGear.h"

#if AP_LANDINGGEAR_ENABLED

#include <AP_Math/AP_Math.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <SITL/SITL.h>
#endif

#if defined(APM_BUILD_TYPE)
//  - this is just here to encourage the build system to supply the "legacy build defines".  The actual dependecy is in the AP_LandingGear.h and AP_LandingGear_config.h headers
#endif

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_LandingGear::var_info[] = {

    // 0 and 1 used by previous retract and deploy pwm, now replaced with SERVOn_MIN/MAX/REVERSED

    // @Param: ENABLE
    // @DisplayName: Enable landing gear
    // @Description: Enable landing gear control
    // @Values: 0:Disabled, 1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 10, AP_LandingGear, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: STARTUP
    // @DisplayName: Landing Gear Startup position
    // @Description: Landing Gear Startup behaviour control
    // @Values: 0:WaitForPilotInput, 1:Retract, 2:Deploy
    // @User: Standard
    AP_GROUPINFO("STARTUP", 2, AP_LandingGear, _startup_behaviour, (uint8_t)AP_LandingGear::LandingGear_Startup_WaitForPilotInput),

    // @Param: DEPLOY_PIN
    // @DisplayName: Chassis deployment feedback pin
    // @Description: Pin number to use for detection of gear deployment. If set to -1 feedback is disabled. Some common values are given, but see the Wiki's "GPIOs" page for how to determine the pin number for a given autopilot.
    // @Values: -1:Disabled,50:AUX1,51:AUX2,52:AUX3,53:AUX4,54:AUX5,55:AUX6
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("DEPLOY_PIN", 3, AP_LandingGear, _pin_deployed, -1),

    // @Param: DEPLOY_POL
    // @DisplayName: Chassis deployment feedback pin polarity
    // @Description: Polarity for feedback pin. If this is 1 then the pin should be high when gear are deployed. If set to 0 then then deployed gear level is low.
    // @Values: 0:Low,1:High
    // @User: Standard
    AP_GROUPINFO("DEPLOY_POL", 4, AP_LandingGear, _pin_deployed_polarity, 0),

    // @Param: WOW_PIN
    // @DisplayName: Weight on wheels feedback pin
    // @Description: Pin number to use for feedback of weight on wheels condition. If set to -1 feedback is disabled. Some common values are given, but see the Wiki's "GPIOs" page for how to determine the pin number for a given autopilot.
    // @Values: -1:Disabled,50:AUX1,51:AUX2,52:AUX3,53:AUX4,54:AUX5,55:AUX6
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("WOW_PIN", 5, AP_LandingGear, _pin_weight_on_wheels, -1),

    // @Param: WOW_POL
    // @DisplayName: Weight on wheels feedback pin polarity
    // @Description: Polarity for feedback pin. If this is 1 then the pin should be high when there is weight on wheels. If set to 0 then then weight on wheels level is low.
    // @Values: 0:Low,1:High
    // @User: Standard
    AP_GROUPINFO("WOW_POL", 6, AP_LandingGear, _pin_weight_on_wheels_polarity, 0),

    // @Param: DEPLOY_ALT
    // @DisplayName: Landing gear deployment altitude
    // @Description: Altitude where the landing gear will be deployed. This should be lower than the RETRACT_ALT. If zero then altitude is not used for deploying landing gear. Only applies when vehicle is armed.
    // @Units: m
    // @Range: 0 1000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("DEPLOY_ALT", 7, AP_LandingGear, _deploy_alt, 0),

    // @Param: RETRACT_ALT
    // @DisplayName: Landing gear retract altitude
    // @Description: Altitude where the landing gear will be retracted. This should be higher than the DEPLOY_ALT. If zero then altitude is not used for retracting landing gear. Only applies when vehicle is armed.
    // @Units: m
    // @Range: 0 1000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RETRACT_ALT", 8, AP_LandingGear, _retract_alt, 0),

    // @Param: OPTIONS
    // @DisplayName: Landing gear auto retract/deploy options
    // @Description: Options to retract or deploy landing gear in Auto or Guided mode
    // @Bitmask: 0:Retract after Takeoff,1:Deploy during Land
    // @User: Standard
    AP_GROUPINFO("OPTIONS", 9, AP_LandingGear, _options, 3),

    // index 10 is enable, placed at the top of the table

    AP_GROUPEND
};

AP_LandingGear *AP_LandingGear::_singleton;

/// initialise state of landing gear
void AP_LandingGear::init()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (AP::sitl()->wow_pin > 0) {
        _pin_weight_on_wheels.set_and_default(AP::sitl()->wow_pin);
        _pin_weight_on_wheels_polarity.set_and_default(1);
    }
#endif

    if (!_enable.configured() && (SRV_Channels::function_assigned(SRV_Channel::k_landing_gear_control) || 
            (_pin_deployed > 0) || (_pin_weight_on_wheels > 0))) {
        // if not configured set enable param if output servo or sense pins are defined
        _enable.set_and_save(1);
    }

    if (_pin_deployed != -1) {
        hal.gpio->pinMode(_pin_deployed, HAL_GPIO_INPUT);
        // set pullup/pulldown to default to non-deployed state
        hal.gpio->write(_pin_deployed, !_pin_deployed_polarity);
        log_wow_state(wow_state_current);
    }

    if (_pin_weight_on_wheels != -1) {
        hal.gpio->pinMode(_pin_weight_on_wheels, HAL_GPIO_INPUT);
        // set pullup/pulldown to default to flying state
        hal.gpio->write(_pin_weight_on_wheels, !_pin_weight_on_wheels_polarity);
        log_wow_state(wow_state_current);
    }

    switch ((enum LandingGearStartupBehaviour)_startup_behaviour.get()) {
        default:
        case LandingGear_Startup_WaitForPilotInput:
            // do nothing
            break;
        case LandingGear_Startup_Retract:
            retract();
            break;
        case LandingGear_Startup_Deploy:
            deploy();
            break;
    }
}

/// set landing gear position to retract, deploy or deploy-and-keep-deployed
void AP_LandingGear::set_position(LandingGearCommand cmd)
{
    switch (cmd) {
        case LandingGear_Retract:
            retract();
            break;
        case LandingGear_Deploy:
            deploy();
            break;
    }
}

/// deploy - deploy landing gear
void AP_LandingGear::deploy()
{
    if (!_enable) {
        return;
    }

    // set servo PWM to deployed position
    SRV_Channels::set_output_limit(SRV_Channel::k_landing_gear_control, SRV_Channel::Limit::MAX);

    // send message only if output has been configured
    if (!_deployed &&
        SRV_Channels::function_assigned(SRV_Channel::k_landing_gear_control)) {
        gcs().send_text(MAV_SEVERITY_INFO, "LandingGear: DEPLOY");
    }

    // set deployed flag
    _deployed = true;
    _have_changed = true;
    LOGGER_WRITE_EVENT(LogEvent::LANDING_GEAR_DEPLOYED);
}

/// retract - retract landing gear
void AP_LandingGear::retract()
{
    if (!_enable) {
        return;
    }

    // set servo PWM to retracted position
    SRV_Channels::set_output_limit(SRV_Channel::k_landing_gear_control, SRV_Channel::Limit::MIN);

    // reset deployed flag
    _deployed = false;
    _have_changed = true;
    LOGGER_WRITE_EVENT(LogEvent::LANDING_GEAR_RETRACTED);

    // send message only if output has been configured
    if (SRV_Channels::function_assigned(SRV_Channel::k_landing_gear_control)) {
        gcs().send_text(MAV_SEVERITY_INFO, "LandingGear: RETRACT");
    }
}

bool AP_LandingGear::deployed()
{
    if (_pin_deployed == -1) {
        return _deployed;
    } else {
        return hal.gpio->read(_pin_deployed) == _pin_deployed_polarity;
    }
}

AP_LandingGear::LG_WOW_State AP_LandingGear::get_wow_state()
{
    return wow_state_current;
}

AP_LandingGear::LG_LandingGear_State AP_LandingGear::get_state()
{
    return gear_state_current;
}

uint32_t AP_LandingGear::get_gear_state_duration_ms() const
{
    if (last_gear_event_ms == 0) {
        return 0;
    }

    return AP_HAL::millis() - last_gear_event_ms;
}

uint32_t AP_LandingGear::get_wow_state_duration_ms() const
{
    if (last_wow_event_ms == 0) {
        return 0;
    }

    return AP_HAL::millis() - last_wow_event_ms;
}

void AP_LandingGear::update(float height_above_ground_m)
{
    if (_pin_weight_on_wheels == -1) {
        last_wow_event_ms = 0;
        wow_state_current = LG_WOW_UNKNOWN;
    } else {
        LG_WOW_State wow_state_new = hal.gpio->read(_pin_weight_on_wheels) == _pin_weight_on_wheels_polarity ? LG_WOW : LG_NO_WOW;

        if (wow_state_new != wow_state_current) {
            // we changed states, lets note the time.
            last_wow_event_ms = AP_HAL::millis();
            log_wow_state(wow_state_new);
        }

        wow_state_current = wow_state_new;
    }

    if (_pin_deployed == -1) {
        last_gear_event_ms = 0;

        // If there was no pilot input and state is still unknown - leave it as it is
        if (gear_state_current != LG_UNKNOWN) {
            gear_state_current = (_deployed == true ? LG_DEPLOYED : LG_RETRACTED);
        }
    } else {
        LG_LandingGear_State gear_state_new;
        
        if (_deployed) {
            gear_state_new = (deployed() == true ? LG_DEPLOYED : LG_DEPLOYING);
        } else {
            gear_state_new = (deployed() == false ? LG_RETRACTED : LG_RETRACTING);
        }

        if (gear_state_new != gear_state_current) {
            // we changed states, lets note the time.
            last_gear_event_ms = AP_HAL::millis();
            
            log_wow_state(wow_state_current);
        }

        gear_state_current = gear_state_new;
    }

    /*
      check for height based triggering
     */
    int16_t alt_m = constrain_int16(height_above_ground_m, 0, INT16_MAX);

    if (hal.util->get_soft_armed()) {
        // only do height based triggering when armed
        if ((!_deployed || !_have_changed) &&
            _deploy_alt > 0 &&
            alt_m <= _deploy_alt &&
            _last_height_above_ground > _deploy_alt) {
            deploy();
        }
        if ((_deployed || !_have_changed) &&
            _retract_alt > 0 &&
            _retract_alt >= _deploy_alt &&
            alt_m >= _retract_alt &&
            _last_height_above_ground < _retract_alt) {
            retract();
        }
    }

    _last_height_above_ground = alt_m;
}

#if HAL_LOGGING_ENABLED
// log weight on wheels state
void AP_LandingGear::log_wow_state(LG_WOW_State state)
{
    AP::logger().Write("LGR", "TimeUS,LandingGear,WeightOnWheels", "Qbb",
                                           AP_HAL::micros64(),
                                           (int8_t)gear_state_current, (int8_t)state);
}
#endif

bool AP_LandingGear::check_before_land(void)
{
    // If the landing gear state is not known (most probably as it is not used)
    if (get_state() == LG_UNKNOWN) {
        return true;
    }

    // If the landing gear was not used - return true, otherwise - check for deployed
    return (get_state() == LG_DEPLOYED);
}

// retract after takeoff if configured via the OPTIONS parameter
void AP_LandingGear::retract_after_takeoff()
{
    if (_options.get() & (uint16_t)Option::RETRACT_AFTER_TAKEOFF) {
        retract();
    }
}

// deploy for landing if configured via the OPTIONS parameter
void AP_LandingGear::deploy_for_landing()
{
    if (_options.get() & (uint16_t)Option::DEPLOY_DURING_LANDING) {
        deploy();
    }
}

#endif
