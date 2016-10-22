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
#include "SRV_Channel.h"
#include "RC_Channel.h"

extern const AP_HAL::HAL& hal;

SRV_Channel *SRV_Channels::channels;
bool SRV_Channels::disabled_passthrough;
bool SRV_Channels::initialised;
Bitmask SRV_Channels::function_mask{SRV_Channel::k_nr_aux_servo_functions};
SRV_Channels::srv_function SRV_Channels::functions[SRV_Channel::k_nr_aux_servo_functions];
SRV_Channel::servo_mask_t SRV_Channel::have_pwm_mask;

const AP_Param::GroupInfo SRV_Channel::var_info[] = {
    // @Param: MIN
    // @DisplayName: Minimum PWM
    // @Description: minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
    // @Units: pwm
    // @Range: 800 2200
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("MIN",  1, SRV_Channel, servo_min, 1100),

    // @Param: MAX
    // @DisplayName: Maximum PWM
    // @Description: maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
    // @Units: pwm
    // @Range: 800 2200
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("MAX",  2, SRV_Channel, servo_max, 1900),

    // @Param: TRIM
    // @DisplayName: Trim PWM
    // @Description: Trim PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
    // @Units: pwm
    // @Range: 800 2200
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("TRIM",  3, SRV_Channel, servo_trim, 1500),
    
    // @Param: REVERSED
    // @DisplayName: Servo reverse
    // @Description: Reverse servo operation. Set to 0 for normal operation. Set to 1 to reverse this channel.
    // @Values: 0:Normal,1:Reversed
    // @User: Standard
    AP_GROUPINFO("REVERSED",  4, SRV_Channel, reversed, 0),

    // @Param: FUNCTION
    // @DisplayName: Servo output function
    // @Description: Function assigned to this servo. Seeing this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function
    // @Values: 0:Disabled,1:RCPassThru,2:Flap,3:Flap_auto,4:Aileron,6:mount_pan,7:mount_tilt,8:mount_roll,9:mount_open,10:camera_trigger,11:release,12:mount2_pan,13:mount2_tilt,14:mount2_roll,15:mount2_open,16:DifferentialSpoiler1,17:DifferentialSpoiler2,18:AileronWithInput,19:Elevator,20:ElevatorWithInput,21:Rudder,24:Flaperon1,25:Flaperon2,26:GroundSteering,27:Parachute,28:EPM,29:LandingGear,30:EngineRunEnable,31:HeliRSC,32:HeliTailRSC,33:Motor1,34:Motor2,35:Motor3,36:Motor4,37:Motor5,38:Motor6,39:Motor7,40:Motor8,51:RCIN1,52:RCIN2,53:RCIN3,54:RCIN4,55:RCIN5,56:RCIN6,57:RCIN7,58:RCIN8,59:RCIN9,60:RCIN10,61:RCIN11,62:RCIN12,63:RCIN13,64:RCIN14,65:RCIN15,66:RCIN16,67:Ignition,68:Choke,69:Starter,70:Throttle
    // @User: Standard
    AP_GROUPINFO("FUNCTION",  5, SRV_Channel, function, 0),
    
    AP_GROUPEND
};


const AP_Param::GroupInfo SRV_Channels::var_info[] = {
    // @Group: 1_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[0], "1_",  1, SRV_Channels, SRV_Channel),

    // @Group: 2_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[1], "2_",  2, SRV_Channels, SRV_Channel),

    // @Group: 3_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[2], "3_",  3, SRV_Channels, SRV_Channel),

    // @Group: 4_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[3], "4_",  4, SRV_Channels, SRV_Channel),

    // @Group: 5_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[4], "5_",  5, SRV_Channels, SRV_Channel),

    // @Group: 6_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[5], "6_",  6, SRV_Channels, SRV_Channel),

    // @Group: 7_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[6], "7_",  7, SRV_Channels, SRV_Channel),

    // @Group: 8_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[7], "8_",  8, SRV_Channels, SRV_Channel),

    // @Group: 9_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[8], "9_",  9, SRV_Channels, SRV_Channel),

    // @Group: 10_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[9], "10_",  10, SRV_Channels, SRV_Channel),

    // @Group: 11_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[10], "11_",  11, SRV_Channels, SRV_Channel),

    // @Group: 12_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[11], "12_",  12, SRV_Channels, SRV_Channel),

    // @Group: 13_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[12], "13_",  13, SRV_Channels, SRV_Channel),

    // @Group: 14_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[13], "14_",  14, SRV_Channels, SRV_Channel),

    // @Group: 15_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[14], "15_",  15, SRV_Channels, SRV_Channel),

    // @Group: 16_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[15], "16_",  16, SRV_Channels, SRV_Channel),
    
    // @Param: _AUTO_TRIM
    // @DisplayName: Automatic servo trim
    // @Description: This enables automatic servo trim in flight. Servos will be trimed in stabilized flight modes when the aircraft is close to level. Changes to servo trim will be saved every 10 seconds and will persist between flights.
    // @Values: 0:Disable,1:Enable
    // @User: Advanced
    AP_GROUPINFO("_AUTO_TRIM",  17, SRV_Channels, auto_trim, 0),
    
    AP_GROUPEND
};

SRV_Channel::SRV_Channel(void)
{
    AP_Param::setup_object_defaults(this, var_info);
    // start with all pwm at zero
    have_pwm_mask = ~uint16_t(0);
}


/*
  constructor
 */
SRV_Channels::SRV_Channels(const RCMapper &_rcmap) :
    rcmap(_rcmap)
{
    channels = obj_channels;
    
    // set defaults from the parameter table
    AP_Param::setup_object_defaults(this, var_info);

    // setup ch_num on channels
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        channels[i].ch_num = i;
    }
}

/*
  save adjusted trims
 */
void SRV_Channels::save_trim(void)
{
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        if (trimmed_mask & (1U<<i)) {
            channels[i].servo_trim.set_and_save(channels[i].servo_trim.get());
        }
    }
    trimmed_mask = 0;
}

// convert a 0..range_max to a pwm
uint16_t SRV_Channel::pwm_from_range(int16_t scaled_value) const
{
    if (servo_max <= servo_min || high_out == 0) {
        return servo_min;
    }
    if (scaled_value >= high_out) {
        scaled_value = high_out;
    }
    if (scaled_value < 0) {
        scaled_value = 0;
    }
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
    uint16_t pwm;
    if (type_angle) {
        pwm = pwm_from_angle(output_scaled);
    } else {
        pwm = pwm_from_range(output_scaled);
    }
    set_output_pwm(pwm);
}

void SRV_Channels::output_trim_all(void)
{
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        channels[i].set_output_pwm(channels[i].servo_trim);
    }
}

void SRV_Channels::setup_failsafe_trim_all(void)
{
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        hal.rcout->set_failsafe_pwm(1U<<channels[i].ch_num, channels[i].servo_trim);
    }
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

uint16_t SRV_Channel::get_limit_pwm(LimitValue limit) const
{
    switch (limit) {
    case SRV_CHANNEL_LIMIT_TRIM:
        return servo_trim;
    case SRV_CHANNEL_LIMIT_MIN:
        return servo_min;
    case SRV_CHANNEL_LIMIT_MAX:
        return servo_max;
    case SRV_CHANNEL_LIMIT_ZERO_PWM:
    default:
        return 0;
    }
}

/*
  run calc_pwm for all channels
 */
void SRV_Channels::calc_pwm(void)
{
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        channels[i].calc_pwm(functions[channels[i].function].output_scaled);
    }
}
