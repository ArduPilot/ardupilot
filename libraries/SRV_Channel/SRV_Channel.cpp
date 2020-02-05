/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  SRV_Channel.cpp - object to separate input and output channel
  ranges, trim and reversal
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_Math/AP_Math.h>
#include "SRV_Channel.h"

extern const AP_HAL::HAL& hal;

SRV_Channel::servo_mask_t SRV_Channel::have_pwm_mask;

const AP_Param::GroupInfo SRV_Channel::var_info[] = {
    // @Param: MIN
    // @DisplayName: Minimum PWM
    // @Description: minimum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
    // @Units: PWM
    // @Range: 500 2200
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("MIN",  1, SRV_Channel, servo_min, 1100),

    // @Param: MAX
    // @DisplayName: Maximum PWM
    // @Description: maximum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
    // @Units: PWM
    // @Range: 800 2200
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("MAX",  2, SRV_Channel, servo_max, 1900),

    // @Param: TRIM
    // @DisplayName: Trim PWM
    // @Description: Trim PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
    // @Units: PWM
    // @Range: 800 2200
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("TRIM",  3, SRV_Channel, servo_trim, 1500),

    // @Param: REVERSED
    // @DisplayName: Servo reverse
    // @Description: Reverse servo operation. Set to 0 for normal operation. Set to 1 to reverse this output channel.
    // @Values: 0:Normal,1:Reversed
    // @User: Standard
    AP_GROUPINFO("REVERSED",  4, SRV_Channel, reversed, 0),

    // @Param: FUNCTION
    // @DisplayName: Servo output function
    // @Description: Function assigned to this servo. Seeing this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function
    // @Values: 0:Disabled,1:RCPassThru,2:Flap,3:Flap_auto,4:Aileron,6:mount_pan,7:mount_tilt,8:mount_roll,9:mount_open,10:camera_trigger,12:mount2_pan,13:mount2_tilt,14:mount2_roll,15:mount2_open,16:DifferentialSpoilerLeft1,17:DifferentialSpoilerRight1,19:Elevator,21:Rudder,22:SprayerPump,23:SprayerSpinner,24:FlaperonLeft,25:FlaperonRight,26:GroundSteering,27:Parachute,28:Gripper,29:LandingGear,30:EngineRunEnable,31:HeliRSC,32:HeliTailRSC,33:Motor1,34:Motor2,35:Motor3,36:Motor4,37:Motor5,38:Motor6,39:Motor7,40:Motor8,41:MotorTilt,51:RCIN1,52:RCIN2,53:RCIN3,54:RCIN4,55:RCIN5,56:RCIN6,57:RCIN7,58:RCIN8,59:RCIN9,60:RCIN10,61:RCIN11,62:RCIN12,63:RCIN13,64:RCIN14,65:RCIN15,66:RCIN16,67:Ignition,69:Starter,70:Throttle,71:TrackerYaw,72:TrackerPitch,73:ThrottleLeft,74:ThrottleRight,75:tiltMotorLeft,76:tiltMotorRight,77:ElevonLeft,78:ElevonRight,79:VTailLeft,80:VTailRight,81:BoostThrottle,82:Motor9,83:Motor10,84:Motor11,85:Motor12,86:DifferentialSpoilerLeft2,87:DifferentialSpoilerRight2,89:Main Sail,90:CameraISO,91:CameraAperture,92:CameraFocus,93:CameraShutterSpeed,94:Script 1,95:Script 2,96:Script 3,97:Script 4,98:Script 5,99:Script 6,100:Script 7,101:Script 8,102:Script 9,103:Script 10,104:Script 11,105:Script 12,106:Script 13,107:Script 14,108:Script 15,109:Script 16,120:NeoPixel1,121:NeoPixel2,122:NeoPixel3,123:NeoPixel4,124:RateRoll,125:RatePitch,126:RateThrust,127:RateYaw,128:Wing Sail Elevator
    // @User: Standard
    AP_GROUPINFO("FUNCTION",  5, SRV_Channel, function, 0),

    AP_GROUPEND
};


SRV_Channel::SRV_Channel(void)
{
    AP_Param::setup_object_defaults(this, var_info);
    // start with all pwm at zero
    have_pwm_mask = ~uint16_t(0);
}

// convert a 0..range_max to a pwm
uint16_t SRV_Channel::pwm_from_range(int16_t scaled_value) const
{
    if (servo_max <= servo_min || high_out == 0) {
        return servo_min;
    }
    scaled_value = constrain_int16(scaled_value, 0, high_out);
    if (reversed) {
        scaled_value = high_out - scaled_value;
    }
    return servo_min + ((int32_t)scaled_value * (int32_t)(servo_max - servo_min)) / (int32_t)high_out;
}

// convert a -angle_max..angle_max to a pwm
uint16_t SRV_Channel::pwm_from_angle(int16_t scaled_value) const
{
    if (reversed) {
        scaled_value = -scaled_value;
    }
    scaled_value = constrain_int16(scaled_value, -high_out, high_out);
    if (scaled_value > 0) {
        return servo_trim + ((int32_t)scaled_value * (int32_t)(servo_max - servo_trim)) / (int32_t)high_out;
    } else {
        return servo_trim - (-(int32_t)scaled_value * (int32_t)(servo_trim - servo_min)) / (int32_t)high_out;
    }
}

void SRV_Channel::calc_pwm(int16_t output_scaled)
{
    if (have_pwm_mask & (1U<<ch_num)) {
        return;
    }

    // check for E - stop
    if (SRV_Channel::should_e_stop(get_function()) && SRV_Channels::emergency_stop) {
        output_scaled = 0;
    }

    uint16_t pwm;
    if (type_angle) {
        pwm = pwm_from_angle(output_scaled);
    } else {
        pwm = pwm_from_range(output_scaled);
    }
    set_output_pwm(pwm);
}

void SRV_Channel::set_output_pwm(uint16_t pwm)
{
    output_pwm = pwm;
    have_pwm_mask |= (1U<<ch_num);
}

// set angular range of scaled output
void SRV_Channel::set_angle(int16_t angle)
{
    type_angle = true;
    high_out = angle;    
    type_setup = true;
}

// set range of scaled output
void SRV_Channel::set_range(uint16_t high)
{
    type_angle = false;
    high_out = high;
    type_setup = true;
}

/*
  get normalised output from -1 to 1, assuming 0 at mid point of servo_min/servo_max
 */
float SRV_Channel::get_output_norm(void)
{
    uint16_t mid = (servo_max + servo_min) / 2;
    float ret;
    if (mid <= servo_min) {
        return 0;
    }
    if (output_pwm < mid) {
        ret = (float)(output_pwm - mid) / (float)(mid - servo_min);
    } else if (output_pwm > mid) {
        ret = (float)(output_pwm - mid) / (float)(servo_max  - mid);
    } else {
        ret = 0;
    }
    if (get_reversed()) {
           ret = -ret;
    }
    return ret;
}

uint16_t SRV_Channel::get_limit_pwm(Limit limit) const
{
    switch (limit) {
    case Limit::TRIM:
        return servo_trim;
    case Limit::MIN:
        return reversed?servo_max:servo_min;
    case Limit::MAX:
        return reversed?servo_min:servo_max;
    case Limit::ZERO_PWM:
        return 0;
    }
    return 0;
}

// return true if function is for a multicopter motor
bool SRV_Channel::is_motor(SRV_Channel::Aux_servo_function_t function)
{
    return ((function >= SRV_Channel::k_motor1 && function <= SRV_Channel::k_motor8) ||
            (function >= SRV_Channel::k_motor9 && function <= SRV_Channel::k_motor12));
}

// return true if function is for anything that should be stopped in a e-stop situation, ie is dangerous
bool SRV_Channel::should_e_stop(SRV_Channel::Aux_servo_function_t function)
{
    return ((function >= SRV_Channel::k_heli_rsc && function <= SRV_Channel::k_motor8) ||
            function == SRV_Channel::k_starter || function == SRV_Channel::k_throttle ||
            function == SRV_Channel::k_throttleLeft || function == SRV_Channel::k_throttleRight ||
            (function >= SRV_Channel::k_boost_throttle && function <= SRV_Channel::k_motor12) ||
            function == k_engine_run_enable);
}
