#include "AP_LandingGear.h"
#include <AP_Relay/AP_Relay.h>
#include <AP_Math/AP_Math.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_HAL/AP_HAL.h>
#include <DataFlash/DataFlash.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_LandingGear::var_info[] = {

    // @Param: SERVO_RTRACT
    // @DisplayName: Landing Gear Servo Retracted PWM Value
    // @Description: Servo PWM value in microseconds when landing gear is retracted
    // @Range: 1000 2000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SERVO_RTRACT", 0, AP_LandingGear, _servo_retract_pwm, AP_LANDINGGEAR_SERVO_RETRACT_PWM_DEFAULT),

    // @Param: SERVO_DEPLOY
    // @DisplayName: Landing Gear Servo Deployed PWM Value
    // @Description: Servo PWM value in microseconds when landing gear is deployed
    // @Range: 1000 2000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SERVO_DEPLOY", 1, AP_LandingGear, _servo_deploy_pwm, AP_LANDINGGEAR_SERVO_DEPLOY_PWM_DEFAULT),

    // @Param: STARTUP
    // @DisplayName: Landing Gear Startup position
    // @Description: Landing Gear Startup behaviour control
    // @Values: 0:WaitForPilotInput, 1:Retract, 2:Deploy
    // @User: Standard
    AP_GROUPINFO("STARTUP", 2, AP_LandingGear, _startup_behaviour, (uint8_t)AP_LandingGear::LandingGear_Startup_WaitForPilotInput),
    
    // @Param: DEPLOY_PIN
    // @DisplayName: Chassis deployment feedback pin
    // @Description: Pin number to use for detection of gear deployment. If set to -1 feedback is disabled.
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
    // @Description: Pin number to use for feedback of weight on wheels condition. If set to -1 feedback is disabled.
    // @Values: -1:Disabled,50:AUX1,51:AUX2,52:AUX3,53:AUX4,54:AUX5,55:AUX6
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("WOW_PIN", 5, AP_LandingGear, _pin_weight_on_wheels, DEFAULT_PIN_WOW),

    // @Param: WOW_POL
    // @DisplayName: Weight on wheels feedback pin polarity
    // @Description: Polarity for feedback pin. If this is 1 then the pin should be high when there is weight on wheels. If set to 0 then then weight on wheels level is low.
    // @Values: 0:Low,1:High
    // @User: Standard
    AP_GROUPINFO("WOW_POL", 6, AP_LandingGear, _pin_weight_on_wheels_polarity, DEFAULT_PIN_WOW_POL),
    
    AP_GROUPEND
};

AP_LandingGear *AP_LandingGear::_singleton;

/// initialise state of landing gear
void AP_LandingGear::init()
{
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
    // set servo PWM to deployed position
    SRV_Channels::set_output_pwm(SRV_Channel::k_landing_gear_control, _servo_deploy_pwm);

    // set deployed flag
    _deployed = true;
}

/// retract - retract landing gear
void AP_LandingGear::retract()
{
    // set servo PWM to retracted position
    SRV_Channels::set_output_pwm(SRV_Channel::k_landing_gear_control, _servo_retract_pwm);

    // reset deployed flag
    _deployed = false;
}

bool AP_LandingGear::deployed()
{
    if (_pin_deployed == -1) {
        return _deployed;
    } else {
        return hal.gpio->read(_pin_deployed) == _pin_deployed_polarity ? true : false;
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

uint32_t AP_LandingGear::get_gear_state_duration_ms()
{
    if (last_gear_event_ms == 0) {
        return 0;
    }

    return AP_HAL::millis() - last_gear_event_ms;
}

uint32_t AP_LandingGear::get_wow_state_duration_ms()
{
    if (last_wow_event_ms == 0) {
        return 0;
    }

    return AP_HAL::millis() - last_wow_event_ms;
}

void AP_LandingGear::update()
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
        gear_state_current = (_deployed == true ? LG_DEPLOYED : LG_RETRACTED);
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
}

// log weight on wheels state
void AP_LandingGear::log_wow_state(LG_WOW_State state)
{
    DataFlash_Class::instance()->Log_Write("LGR", "TimeUS,LandingGear,WeightOnWheels", "Qbb",
                                           AP_HAL::micros64(),
                                           (int8_t)gear_state_current, (int8_t)state);
}
