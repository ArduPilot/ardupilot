#include "AP_LandingGear.h"
#include <AP_Relay/AP_Relay.h>
#include <AP_Math/AP_Math.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_HAL/AP_HAL.h>

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

    AP_GROUPEND
};

/// initialise state of landing gear
void AP_LandingGear::init()
{
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
            if (!_deploy_lock) {
                retract();
            }
            break;
        case LandingGear_Deploy:
            deploy();
            _deploy_lock = false;
            break;
        case LandingGear_Deploy_And_Keep_Deployed:
            deploy();
            _deploy_lock = true;
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
