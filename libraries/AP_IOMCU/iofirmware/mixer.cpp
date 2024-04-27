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
  mixer for failsafe operation when FMU is dead
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <SRV_Channel/SRV_Channel.h>
#include "iofirmware.h"

#define ANGLE_SCALE ((int32_t)4500)
#define RANGE_SCALE ((int32_t)1000)

/*
  return a RC input value scaled from -4500 to 4500
 */
int16_t AP_IOMCU_FW::mix_input_angle(uint8_t channel, uint16_t radio_in) const
{
    const uint16_t &rc_min = mixing.rc_min[channel];
    const uint16_t &rc_max = mixing.rc_max[channel];
    const uint16_t &rc_trim = mixing.rc_trim[channel];
    const uint16_t &reversed = mixing.rc_reversed[channel];

    int16_t ret = 0;
    if (radio_in > rc_trim && rc_max != rc_trim) {
        ret = (ANGLE_SCALE * (int32_t)(radio_in - rc_trim)) / (int32_t)(rc_max  - rc_trim);
    } else if (radio_in < rc_trim && rc_trim != rc_min) {
        ret = (ANGLE_SCALE * (int32_t)(radio_in - rc_trim)) / (int32_t)(rc_trim - rc_min);
    }
    if (reversed) {
        ret = -ret;
    }
    return ret;
}

/*
  return a RC input value scaled from 0 to 1000
 */
int16_t AP_IOMCU_FW::mix_input_range(uint8_t channel, uint16_t radio_in) const
{
    const uint16_t &rc_min = mixing.rc_min[channel];
    const uint16_t &rc_max = mixing.rc_max[channel];
    const uint16_t &reversed = mixing.rc_reversed[channel];

    int16_t ret = 0;
    if (radio_in > rc_max) {
        ret = RANGE_SCALE;
    } else if (radio_in < rc_min) {
        ret = -RANGE_SCALE;
    } else {
        ret = (RANGE_SCALE * (int32_t)(radio_in - rc_min)) / (int32_t)(rc_max  - rc_min);
    }
    if (reversed) {
        ret = -ret;
    }
    return ret;
}

/*
  return an output pwm giving an angle for a servo channel
 */
uint16_t AP_IOMCU_FW::mix_output_angle(uint8_t channel, int16_t angle) const
{
    const uint16_t &srv_min = mixing.servo_min[channel];
    const uint16_t &srv_max = mixing.servo_max[channel];
    const uint16_t &srv_trim = mixing.servo_trim[channel];
    const uint16_t &reversed = mixing.servo_reversed[channel];
    if (reversed) {
        angle = -angle;
    }
    angle = constrain_int16(angle, -ANGLE_SCALE, ANGLE_SCALE);
    if (angle > 0) {
        return srv_trim + ((int32_t)angle * (int32_t)(srv_max - srv_trim)) / ANGLE_SCALE;
    }
    return srv_trim - (-(int32_t)angle * (int32_t)(srv_trim - srv_min)) / ANGLE_SCALE;
}

/*
  return an output pwm giving an range for a servo channel
 */
uint16_t AP_IOMCU_FW::mix_output_range(uint8_t channel, int16_t value) const
{
    const uint16_t &srv_min = mixing.servo_min[channel];
    const uint16_t &srv_max = mixing.servo_max[channel];
    const uint16_t &reversed = mixing.servo_reversed[channel];
    value = constrain_int16(value, 0, RANGE_SCALE);
    if (reversed) {
        value = RANGE_SCALE - value;
    }
    return srv_min + ((int32_t)value * (int32_t)(srv_max - srv_min)) / RANGE_SCALE;
}


/*
  elevon and vtail mixer
 */
int16_t AP_IOMCU_FW::mix_elevon_vtail(int16_t angle1, int16_t angle2, bool first_output) const
{
    if (first_output) {
        return (angle2 - angle1) * mixing.mixing_gain / 1000;
    }
    return (angle1 + angle2) * mixing.mixing_gain / 1000;
}

/*
  run mixer. This is used when FMU is not providing inputs, or when
  the OVERRIDE_CHAN is high. It allows for manual fixed wing flight
 */
void AP_IOMCU_FW::run_mixer(void)
{
    int16_t rcin[4] = {0, 0, 0, 0};
    int16_t &roll = rcin[0];
    int16_t &pitch = rcin[1];
    int16_t &throttle = rcin[2];
    int16_t &rudder = rcin[3];

    // get RC input angles
    if (rc_input.flags_rc_ok) {
        for (uint8_t i=0;i<4; i++) {
            if (mixing.rc_channel[i] > 0 && mixing.rc_channel[i] <= IOMCU_MAX_RC_CHANNELS) {
                uint8_t chan = mixing.rc_channel[i]-1;
                if (i == 2 && !mixing.throttle_is_angle) {
                    rcin[i] = mix_input_range(i, rc_input.pwm[chan]);
                } else {
                    rcin[i] = mix_input_angle(i, rc_input.pwm[chan]);
                }
            }
        }
    }

    for (uint8_t i=0; i<IOMCU_MAX_RC_CHANNELS; i++) {
        SRV_Channel::Aux_servo_function_t function = (SRV_Channel::Aux_servo_function_t)mixing.servo_function[i];
        uint16_t &pwm = reg_direct_pwm.pwm[i];

        if (mixing.manual_rc_mask & (1U<<i)) {
            // treat as pass-thru if this channel is set in MANUAL_RC_MASK
            function = SRV_Channel::k_manual;
        }

        switch (function) {
        case SRV_Channel::k_manual:
            pwm = rc_input.pwm[i];
            break;

        case SRV_Channel::k_rcin1 ... SRV_Channel::k_rcin16:
            pwm = rc_input.pwm[(uint8_t)(function - SRV_Channel::k_rcin1)];
            break;

        case SRV_Channel::k_aileron:
        case SRV_Channel::k_aileron_with_input:
        case SRV_Channel::k_flaperon_left:
        case SRV_Channel::k_flaperon_right:
            pwm = mix_output_angle(i, roll);
            break;

        case SRV_Channel::k_elevator:
        case SRV_Channel::k_elevator_with_input:
            pwm = mix_output_angle(i, pitch);
            break;

        case SRV_Channel::k_rudder:
        case SRV_Channel::k_steering:
            pwm = mix_output_angle(i, rudder);
            break;

        case SRV_Channel::k_throttle:
        case SRV_Channel::k_throttleLeft:
        case SRV_Channel::k_throttleRight:
            if (mixing.throttle_is_angle) {
                pwm = mix_output_angle(i, throttle);
            } else {
                pwm = mix_output_range(i, throttle);
            }
            break;

        case SRV_Channel::k_flap:
        case SRV_Channel::k_flap_auto:
            // zero flaps
            pwm = mix_output_range(i, 0);
            break;

        case SRV_Channel::k_elevon_left:
        case SRV_Channel::k_dspoilerLeft1:
        case SRV_Channel::k_dspoilerLeft2:
            // treat differential spoilers as elevons
            pwm = mix_output_angle(i, mix_elevon_vtail(roll, pitch, true));
            break;

        case SRV_Channel::k_elevon_right:
        case SRV_Channel::k_dspoilerRight1:
        case SRV_Channel::k_dspoilerRight2:
            // treat differential spoilers as elevons
            pwm = mix_output_angle(i, mix_elevon_vtail(roll, pitch, false));
            break;

        case SRV_Channel::k_vtail_left:
            pwm = mix_output_angle(i, mix_elevon_vtail(rudder, pitch, false));
            break;

        case SRV_Channel::k_vtail_right:
            pwm = mix_output_angle(i, mix_elevon_vtail(rudder, pitch, true));
            break;

        default:
            break;
        }
    }
}
