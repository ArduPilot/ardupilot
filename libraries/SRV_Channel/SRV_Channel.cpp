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
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include "SRV_Channel.h"

extern const AP_HAL::HAL& hal;

SRV_Channel::servo_mask_t SRV_Channel::have_pwm_mask;

const AP_Param::GroupInfo SRV_Channel::var_info[] = {
    // @Param: MIN
    // @DisplayName: Minimum PWM
    // @Description: minimum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
    // @Units: PWM
    // @Range: 800 2200
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
    // @Description: Function assigned to this servo. Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function
    // @Values: -1:GPIO
    // @Values{Plane, Copter, Rover}: -1:GPIO
    // @Values: 0:Disabled
    // @Values{Plane, Copter, Rover}: 0:Disabled
    // @Values: 1:RCPassThru
    // @Values{Plane, Copter, Rover}: 1:RCPassThru
    // @Values: 2:Flap, 3:FlapAuto
    // @Values{Plane}: 2:Flap,3:FlapAuto
    // @Values: 4:Aileron
    // @Values{Plane}: 4:Aileron
    // @Values: 6:Mount1Yaw,7:Mount1Pitch,8:Mount1Roll,9:Mount1Retract
    // @Values{Plane, Copter, Rover}: 6:Mount1Yaw,7:Mount1Pitch,8:Mount1Roll,9:Mount1Retract
    // @Values: 10:CameraTrigger
    // @Values{Plane, Copter, Rover}: 10:CameraTrigger
    // @Values: 12:Mount2Yaw,13:Mount2Pitch,14:Mount2Roll,15:Mount2Retract
    // @Values{Plane, Copter, Rover}: 12:Mount2Yaw,13:Mount2Pitch,14:Mount2Roll,15:Mount2Retract
    // @Values: 16:DifferentialSpoilerLeft1,17:DifferentialSpoilerRight1
    // @Values{Plane}: 16:DifferentialSpoilerLeft1,17:DifferentialSpoilerRight1
    // @Values: 19:Elevator
    // @Values{Plane}: 19:Elevator
    // @Values: 21:Rudder
    // @Values{Plane}: 21:Rudder
    // @Values: 22:SprayerPump,23:SprayerSpinner
    // @Values{Plane, Copter, Rover}: 22:SprayerPump,23:SprayerSpinner
    // @Values: 24:FlaperonLeft,25:FlaperonRight
    // @Values{Plane}: 24:FlaperonLeft,25:FlaperonRight
    // @Values: 26:GroundSteering
    // @Values{Plane, Rover}: 26:GroundSteering
    // @Values: 27:Parachute
    // @Values{Plane, Copter}: 27:Parachute
    // @Values: 28:Gripper
    // @Values{Plane, Copter, Rover}: 28:Gripper
    // @Values: 29:LandingGear
    // @Values{Plane, Copter}: 29:LandingGear
    // @Values: 30:EngineRunEnable
    // @Values{Plane, Copter}: 30:EngineRunEnable
    // @Values: 31:HeliRSC,32:HeliTailRSC
    // @Values{Copter}: 31:HeliRSC,32:HeliTailRSC
    // @Values: 33:Motor1,34:Motor2,35:Motor3,36:Motor4,37:Motor5,38:Motor6,39:Motor7,40:Motor8
    // @Values{Plane}: 33:Motor1,34:Motor2,35:Motor3,36:Motor4,37:Motor5,38:Motor6,39:Motor7/TailTiltServo,40:Motor8
    // @Values{Copter}: 33:Motor1,34:Motor2,35:Motor3,36:Motor4,37:Motor5,38:Motor6,39:Motor7,40:Motor8
    // @Values{Rover}: 33:Motor1,34:Motor2,35:Motor3,36:Motor4
    // @Values: 41:TiltMotorsFront,45:TiltMotorsRear,46:TiltMotorRearLeft,47:TiltMotorRearRight
    // @Values{Plane}: 41:TiltMotorsFront,45:TiltMotorsRear,46:TiltMotorRearLeft,47:TiltMotorRearRight
    // @Values: 51:RCIN1,52:RCIN2,53:RCIN3,54:RCIN4,55:RCIN5,56:RCIN6,57:RCIN7,58:RCIN8,59:RCIN9,60:RCIN10,61:RCIN11,62:RCIN12,63:RCIN13,64:RCIN14,65:RCIN15,66:RCIN16
    // @Values{Plane, Copter, Rover}: 51:RCIN1,52:RCIN2,53:RCIN3,54:RCIN4,55:RCIN5,56:RCIN6,57:RCIN7,58:RCIN8,59:RCIN9,60:RCIN10,61:RCIN11,62:RCIN12,63:RCIN13,64:RCIN14,65:RCIN15,66:RCIN16
    // @Values: 67:Ignition,69:Starter
    // @Values{Plane}: 67:Ignition,69:Starter
    // @Values: 70:Throttle
    // @Values{Plane, Rover}: 70:Throttle
    // @Values: 71:TrackerYaw,72:TrackerPitch
    // @Values: 73:ThrottleLeft,74:ThrottleRight
    // @Values{Plane, Copter, Rover}: 73:ThrottleLeft,74:ThrottleRight
    // @Values: 75:TiltMotorFrontLeft,76:TiltMotorFrontRight
    // @Values{Plane, Copter}: 75:TiltMotorFrontLeft,76:TiltMotorFrontRight
    // @Values: 77:ElevonLeft,78:ElevonRight
    // @Values{Plane}: 77:ElevonLeft,78:ElevonRight
    // @Values: 79:VTailLeft,80:VTailRight
    // @Values{Plane}: 79:VTailLeft,80:VTailRight
    // @Values: 81:BoostThrottle
    // @Values{Copter}: 81:BoostThrottle
    // @Values: 82:Motor9,83:Motor10,84:Motor11,85:Motor12
    // @Values{Plane, Copter}: 82:Motor9,83:Motor10,84:Motor11,85:Motor12
    // @Values: 86:DifferentialSpoilerLeft2,87:DifferentialSpoilerRight2
    // @Values{Plane}: 86:DifferentialSpoilerLeft2,87:DifferentialSpoilerRight2
    // @Values: 88:Winch
    // @Values{Copter, Rover}: 88:Winch
    // @Values: 89:Main Sail
    // @Values{Rover}: 89:Main Sail
    // @Values: 90:CameraISO,91:CameraAperture,92:CameraFocus,93:CameraShutterSpeed
    // @Values{Plane, Copter, Rover}: 90:CameraISO,91:CameraAperture,92:CameraFocus,93:CameraShutterSpeed
    // @Values: 94:Script1,95:Script2,96:Script3,97:Script4,98:Script5,99:Script6,100:Script7,101:Script8,102:Script9,103:Script10,104:Script11,105:Script12,106:Script13,107:Script14,108:Script15,109:Script16
    // @Values{Plane, Copter, Rover}: 94:Script1,95:Script2,96:Script3,97:Script4,98:Script5,99:Script6,100:Script7,101:Script8,102:Script9,103:Script10,104:Script11,105:Script12,106:Script13,107:Script14,108:Script15,109:Script16
    // @Values{Plane}: 110:Airbrakes
    // @Values: 120:NeoPixel1,121:NeoPixel2,122:NeoPixel3,123:NeoPixel4
    // @Values{Plane, Copter, Rover}: 120:NeoPixel1,121:NeoPixel2,122:NeoPixel3,123:NeoPixel4
    // @Values: 124:RateRoll,125:RatePitch,126:RateThrust,127:RateYaw
    // @Values{Plane, Copter}: 124:RateRoll,125:RatePitch,126:RateThrust,127:RateYaw
    // @Values: 128:WingSailElevator
    // @Values{Rover}: 128:WingSailElevator
    // @Values: 129:ProfiLED1,130:ProfiLED2,131:ProfiLED3,132:ProfiLEDClock
    // @Values{Plane, Copter, Rover}: 129:ProfiLED1,130:ProfiLED2,131:ProfiLED3,132:ProfiLEDClock
    // @Values: 133:Winch Clutch
    // @Values{Copter, Rover}: 133:Winch Clutch
    // @Values: 134:SERVOn_MIN,135:SERVOn_TRIM,136:SERVOn_MAX
    // @Values{Plane, Copter, Rover}: 134:SERVOn_MIN,135:SERVOn_TRIM,136:SERVOn_MAX
    // @Values: 137:SailMastRotation
    // @Values{Rover}: 137:SailMastRotation
    // @Values: 138:Alarm,139:Alarm Inverted
    // @Values{Plane, Copter, Rover}: 138:Alarm,139:Alarm Inverted
    // @Values: 140:RCIN1Scaled,141:RCIN2Scaled,142:RCIN3Scaled,143:RCIN4Scaled,144:RCIN5Scaled,145:RCIN6Scaled,146:RCIN7Scaled,147:RCIN8Scaled,148:RCIN9Scaled,149:RCIN10Scaled,150:RCIN11Scaled,151:RCIN12Scaled,152:RCIN13Scaled,153:RCIN14Scaled,154:RCIN15Scaled,155:RCIN16Scaled
    // @Values{Plane, Copter, Rover}: 140:RCIN1Scaled,141:RCIN2Scaled,142:RCIN3Scaled,143:RCIN4Scaled,144:RCIN5Scaled,145:RCIN6Scaled,146:RCIN7Scaled,147:RCIN8Scaled,148:RCIN9Scaled,149:RCIN10Scaled,150:RCIN11Scaled,151:RCIN12Scaled,152:RCIN13Scaled,153:RCIN14Scaled,154:RCIN15Scaled,155:RCIN16Scaled
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("FUNCTION",  5, SRV_Channel, function, 0),

    AP_GROUPEND
};


SRV_Channel::SRV_Channel(void)
{
    AP_Param::setup_object_defaults(this, var_info);
    // start with all pwm at zero
    have_pwm_mask = ~uint32_t(0);
}

// convert a 0..range_max to a pwm
uint16_t SRV_Channel::pwm_from_range(float scaled_value) const
{
    if (servo_max <= servo_min || high_out == 0) {
        return servo_min;
    }
    scaled_value = constrain_float(scaled_value, 0, high_out);
    if (reversed) {
        scaled_value = high_out - scaled_value;
    }
    return servo_min + uint16_t( (scaled_value * (float)(servo_max - servo_min)) / (float)high_out );
}

// convert a -angle_max..angle_max to a pwm
uint16_t SRV_Channel::pwm_from_angle(float scaled_value) const
{
    if (high_out == 0) {
        return servo_trim;
    }
    if (reversed) {
        scaled_value = -scaled_value;
    }
    scaled_value = constrain_float(scaled_value, -high_out, high_out);
    if (scaled_value > 0) {
        return servo_trim + uint16_t( (scaled_value * (float)(servo_max - servo_trim)) / (float)high_out);
    } else {
        return servo_trim - uint16_t( (-scaled_value * (float)(servo_trim - servo_min)) / (float)high_out);
    }
}

uint16_t SRV_Channel::pwm_from_scaled_value(float scaled_value) const
{
    if (type_angle) {
        return pwm_from_angle(scaled_value);
    } else {
        return pwm_from_range(scaled_value);
    }
}

void SRV_Channel::calc_pwm(float output_scaled)
{
    if (have_pwm_mask & (1U<<ch_num)) {
        // Note that this allows a set_output_pwm call to override E-Stop!!
        // tricky to fix because we would endup E-stoping to individual SEROVx_MIN not MOT_PWM_MIN on copter
        return;
    }

    // check for E - stop
    bool force = false;
    if (SRV_Channel::should_e_stop(get_function()) && SRV_Channels::emergency_stop) {
        output_scaled = 0.0;
        force = true;
    }

    if (!force && override_active) {
        // don't overwrite a override
        return;
    }

    output_pwm = pwm_from_scaled_value(output_scaled);
}

void SRV_Channel::set_output_pwm(uint16_t pwm, bool force)
{
    if (!override_active || force) {
        output_pwm = pwm;
        have_pwm_mask |= (1U<<ch_num);
    }
}

// set normalised output from -1 to 1, assuming 0 at mid point of servo_min/servo_max
void SRV_Channel::set_output_norm(float value)
{
    // convert normalised value to pwm
    set_output_pwm(pwm_from_scaled_value(value * high_out));
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
    switch (function) {
    case Aux_servo_function_t::k_heli_rsc:  
    case Aux_servo_function_t::k_heli_tail_rsc:
    case Aux_servo_function_t::k_motor1:
    case Aux_servo_function_t::k_motor2:
    case Aux_servo_function_t::k_motor3:
    case Aux_servo_function_t::k_motor4:
    case Aux_servo_function_t::k_motor5:
    case Aux_servo_function_t::k_motor6:
    case Aux_servo_function_t::k_motor7:
    case Aux_servo_function_t::k_motor8:
    case Aux_servo_function_t::k_starter:
    case Aux_servo_function_t::k_throttle:
    case Aux_servo_function_t::k_throttleLeft:
    case Aux_servo_function_t::k_throttleRight:
    case Aux_servo_function_t::k_boost_throttle:
    case Aux_servo_function_t::k_motor9:
    case Aux_servo_function_t::k_motor10:
    case Aux_servo_function_t::k_motor11:
    case Aux_servo_function_t::k_motor12:
    case Aux_servo_function_t::k_engine_run_enable:
        return true;
    default:
        return false;
    }
    return false;
}

// return true if function is for a control surface
bool SRV_Channel::is_control_surface(SRV_Channel::Aux_servo_function_t function)
{
    switch (function)
    {
    case SRV_Channel::Aux_servo_function_t::k_flap:  
    case SRV_Channel::Aux_servo_function_t::k_flap_auto:
    case SRV_Channel::Aux_servo_function_t::k_aileron:
    case SRV_Channel::Aux_servo_function_t::k_dspoilerLeft1:
    case SRV_Channel::Aux_servo_function_t::k_dspoilerLeft2:
    case SRV_Channel::Aux_servo_function_t::k_dspoilerRight1:
    case SRV_Channel::Aux_servo_function_t::k_dspoilerRight2:
    case SRV_Channel::Aux_servo_function_t::k_elevator:
    case SRV_Channel::Aux_servo_function_t::k_rudder:
    case SRV_Channel::Aux_servo_function_t::k_flaperon_left:
    case SRV_Channel::Aux_servo_function_t::k_flaperon_right:
    case SRV_Channel::Aux_servo_function_t::k_elevon_left:
    case SRV_Channel::Aux_servo_function_t::k_elevon_right:
    case SRV_Channel::Aux_servo_function_t::k_vtail_left:
    case SRV_Channel::Aux_servo_function_t::k_vtail_right:
    case SRV_Channel::Aux_servo_function_t::k_airbrake:
        return true;

    default:
        return false;
    }

    return false;
}

// return the motor number of a channel, or -1 if not a motor
// return 0 for first motor
int8_t SRV_Channel::get_motor_num(void) const
{
    const auto k_function = get_function();
    switch (k_function) {
    case k_motor1 ... k_motor8:
        return int8_t(uint16_t(k_function) - k_motor1);
    case k_motor9 ... k_motor12:
        return 8 + int8_t(uint16_t(k_function) - k_motor9);
    default:
        break;
    }
    return -1;
}
