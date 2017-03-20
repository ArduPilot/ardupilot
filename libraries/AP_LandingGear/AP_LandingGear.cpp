#include "AP_LandingGear.h"
#include <AP_Relay/AP_Relay.h>
#include <AP_Math/AP_Math.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_LandingGear::var_info[] = {

    // @Param: SERVO_RTRACT
    // @DisplayName: Landing Gear Servo Retracted PWM Value
    // @Description: Servo PWM value when landing gear is retracted
    // @Range: 1000 2000
    // @Units: pwm
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SERVO_RTRACT", 0, AP_LandingGear, _servo_retract_pwm, AP_LANDINGGEAR_SERVO_RETRACT_PWM_DEFAULT),

    // @Param: SERVO_DEPLOY
    // @DisplayName: Landing Gear Servo Deployed PWM Value
    // @Description: Servo PWM value when landing gear is deployed
    // @Range: 1000 2000
    // @Units: pwm
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SERVO_DEPLOY", 1, AP_LandingGear, _servo_deploy_pwm, AP_LANDINGGEAR_SERVO_DEPLOY_PWM_DEFAULT),

    AP_GROUPEND
};

/// enable - enable or disable land gear retraction
void AP_LandingGear::enable(bool on_off)
{
    _retract_enabled = on_off;
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

/// update - should be called at 10hz
void AP_LandingGear::update()
{
    // if there is a force deploy active, disable retraction, then reset force deploy to consume it
    // gear will be deployed automatically because _retract_enabled is false.
    // this will disable retract switch until it is cycled through deploy position
    if ( _force_deploy){
            enable(false);
            force_deploy(false);
        }

    if (!_retract_enabled) {
        // force deployment if retract is not enabled
        deploy();
        // retract is disabled until switch is placed into deploy position to prevent accidental retraction on bootup if switch was left in retract position
        enable(_command_mode == LandingGear_Deploy);
        return;
    }

    if (_command_mode == LandingGear_Deploy){
        deploy();
    }

    if (_command_mode == LandingGear_Retract){
        retract();
    }    
}
