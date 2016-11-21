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
#include "SRV_Channel.h"
#include "RC_Channel.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo SRV_Channels::var_info[] = {
    // @Param: RNG_ENABLE
    // @DisplayName: Enable servo output ranges
    // @Description: This enables the use of separate servo output ranges from input ranges. 
    // @Values: 0:Disable,1:Enable
    // @User: Advanced
    AP_GROUPINFO_FLAGS("_RNG_ENABLE",  1, SRV_Channels, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: 1_MIN
    // @DisplayName: Servo1 min PWM
    // @Description: servo1 minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
    // @Units: pwm
    // @Range: 800 2200
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("1_MIN",  2, SRV_Channels, servo_min[0], 1100),

    // @Param: 1_MAX
    // @DisplayName: Servo1 max PWM
    // @Description: servo1 maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
    // @Units: pwm
    // @Range: 800 2200
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("1_MAX",  3, SRV_Channels, servo_max[0], 1900),

    // @Param: 1_TRIM
    // @DisplayName: Servo1 trim PWM
    // @Description: servo1 trim PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
    // @Units: pwm
    // @Range: 800 2200
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("1_TRIM",  4, SRV_Channels, servo_trim[0], 1500),
    
    // @Param: 1_REV
    // @DisplayName: Servo1 reverse
    // @Description: Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
    // @Values: -1:Reversed,1:Normal
    // @User: Advanced
    AP_GROUPINFO("1_REV",  5, SRV_Channels, reverse[0], 1),

    // @Param: 2_MIN
    // @DisplayName: Servo1 min PWM
    // @Description: servo1 minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
    // @Units: pwm
    // @Range: 800 2200
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("2_MIN",  6, SRV_Channels, servo_min[1], 1100),

    // @Param: 2_MAX
    // @DisplayName: Servo1 max PWM
    // @Description: servo1 maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
    // @Units: pwm
    // @Range: 800 2200
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("2_MAX",  7, SRV_Channels, servo_max[1], 1900),

    // @Param: 2_TRIM
    // @DisplayName: Servo1 trim PWM
    // @Description: servo1 trim PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
    // @Units: pwm
    // @Range: 800 2200
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("2_TRIM",  8, SRV_Channels, servo_trim[1], 1500),
    
    // @Param: 2_REV
    // @DisplayName: Servo1 reverse
    // @Description: Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
    // @Values: -1:Reversed,1:Normal
    // @User: Advanced
    AP_GROUPINFO("2_REV",  9, SRV_Channels, reverse[1], 1),

    // @Param: 3_MIN
    // @DisplayName: Servo1 min PWM
    // @Description: servo1 minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
    // @Units: pwm
    // @Range: 800 2200
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("3_MIN",  10, SRV_Channels, servo_min[2], 1100),

    // @Param: 3_MAX
    // @DisplayName: Servo1 max PWM
    // @Description: servo1 maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
    // @Units: pwm
    // @Range: 800 2200
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("3_MAX",  11, SRV_Channels, servo_max[2], 1900),

    // @Param: 3_TRIM
    // @DisplayName: Servo1 trim PWM
    // @Description: servo1 trim PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
    // @Units: pwm
    // @Range: 800 2200
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("3_TRIM",  12, SRV_Channels, servo_trim[2], 1500),
    
    // @Param: 3_REV
    // @DisplayName: Servo1 reverse
    // @Description: Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
    // @Values: -1:Reversed,1:Normal
    // @User: Advanced
    AP_GROUPINFO("3_REV",  13, SRV_Channels, reverse[2], 1),

    // @Param: 4_MIN
    // @DisplayName: Servo1 min PWM
    // @Description: servo1 minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
    // @Units: pwm
    // @Range: 800 2200
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("4_MIN",  14, SRV_Channels, servo_min[3], 1100),

    // @Param: 4_MAX
    // @DisplayName: Servo1 max PWM
    // @Description: servo1 maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
    // @Units: pwm
    // @Range: 800 2200
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("4_MAX",  15, SRV_Channels, servo_max[3], 1900),

    // @Param: 4_TRIM
    // @DisplayName: Servo1 trim PWM
    // @Description: servo1 trim PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
    // @Units: pwm
    // @Range: 800 2200
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("4_TRIM",  16, SRV_Channels, servo_trim[3], 1500),
    
    // @Param: 4_REV
    // @DisplayName: Servo1 reverse
    // @Description: Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
    // @Values: -1:Reversed,1:Normal
    // @User: Advanced
    AP_GROUPINFO("4_REV",  17, SRV_Channels, reverse[3], 1),

    // @Param: _AUTO_TRIM
    // @DisplayName: Automatic servo trim
    // @Description: This enables automatic servo trim in flight. Servos will be trimed in stabilized flight modes when the aircraft is close to level. Changes to servo trim will be saved every 10 seconds and will persist between flights.
    // @Values: 0:Disable,1:Enable
    // @User: Advanced
    AP_GROUPINFO("_AUTO_TRIM",  18, SRV_Channels, auto_trim, 0),
    
    AP_GROUPEND
};


/*
  constructor
 */
SRV_Channels::SRV_Channels(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}


// remap a PWM value from a channel in value
uint16_t SRV_Channels::remap_pwm(uint8_t i, uint16_t pwm) const
{
    const RC_Channel *ch = RC_Channel::rc_channel(i);
    if (ch == nullptr) {
        return 0;
    }
    float v = ch->get_radio_out_normalised(pwm);
    uint16_t radio_out;
    if (ch->get_type_out() == RC_CHANNEL_TYPE_RANGE) {
        if (reverse[i] == -1) {
            v = 1 - v;
        }
        radio_out = servo_min[i] + v * (servo_max[i] - servo_min[i]);
    } else {
        if (reverse[i] == -1) {
            v = -v;
        }
        if (v > 0) {
            radio_out = servo_trim[i] + v * (servo_max[i] - servo_trim[i]);
        } else {
            radio_out = servo_trim[i] + v * (servo_trim[i] - servo_min[i]);
        }
    }
    radio_out = constrain_int16(radio_out, servo_min[i], servo_max[i]);
    return radio_out;
}

/*
  remap radio_out values for first 4 servos using SERVO* parameters, if enabled
  This should be called with cork enabled in hal.rcout
 */
void SRV_Channels::remap_servo_output(void)
{
    // cope with SERVO_RNG_ENABLE being changed at runtime. If we
    // don't change the ESC scaling immediately then some ESCs will
    // fire up for one second
    if (last_enable != enable && esc_cal_chan != -1) {
        last_enable = enable;
        set_esc_scaling(esc_cal_chan);
    }
    if (!enable) {
        return;
    }
    for (uint8_t i=0; i<NUM_SERVO_RANGE_CHANNELS; i++) {
        RC_Channel *ch = RC_Channel::rc_channel(i);
        if (ch == nullptr) {
            continue;
        }
        uint16_t radio_out = remap_pwm(i, ch->get_radio_out());
        ch->set_radio_out(radio_out);
    }
}


/*
  set trim values for output channels
 */
void SRV_Channels::set_trim(void)
{
    if (!enable) {
        return;
    }
    for (uint8_t i=0; i<NUM_SERVO_RANGE_CHANNELS; i++) {
        const RC_Channel *ch = RC_Channel::rc_channel(i);
        if (ch == nullptr) {
            continue;
        }
        if (ch->get_type_out() == RC_CHANNEL_TYPE_RANGE) {
            // we don't trim range channels (like throttle)
            continue;
        }
        uint16_t new_trim = remap_pwm(i, ch->get_radio_trim());
        servo_trim[i].set_and_save(new_trim);
    }
}

void SRV_Channels::set_esc_scaling(uint8_t chnum)
{
    esc_cal_chan = chnum;
    if (!enable || chnum >= NUM_SERVO_RANGE_CHANNELS) {
        const RC_Channel *ch = RC_Channel::rc_channel(chnum);
        hal.rcout->set_esc_scaling(ch->get_radio_min(), ch->get_radio_max());
    } else {
        hal.rcout->set_esc_scaling(servo_min[chnum], servo_max[chnum]);
    }
}

/*
  auto-adjust channel trim from an integrator value. Positive v means
  adjust trim up. Negative means decrease
 */
void SRV_Channels::adjust_trim(uint8_t chnum, float v)
{
    if (reverse[chnum] == -1) {
        v = -v;
    }
    uint16_t new_trim = servo_trim[chnum];
    float trim_scaled = float(servo_trim[chnum] - servo_min[chnum]) / (servo_max[chnum] - servo_min[chnum]);
    if (v > 0 && trim_scaled < 0.6f) {
        new_trim++;
    } else if (v < 0  && trim_scaled > 0.4f) {
        new_trim--;
    } else {
        return;
    }
    servo_trim[chnum].set(new_trim);

    trimmed_mask |= 1U<<chnum;
}

/*
  save adjusted trims
 */
void SRV_Channels::save_trim(void)
{
    for (uint8_t i=0; i<NUM_SERVO_RANGE_CHANNELS; i++) {
        if (trimmed_mask & (1U<<i)) {
            servo_trim[i].set_and_save(servo_trim[i].get());
        }
    }
    trimmed_mask = 0;
}
