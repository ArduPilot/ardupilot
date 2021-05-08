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
#include <AP_HAL/AP_HAL.h>
#ifdef HAL_PERIPH_ENABLE_RC_OUT
#include "AP_Periph.h"

// magic value from UAVCAN driver packet
// dsdl/uavcan/equipment/esc/1030.RawCommand.uavcan
// Raw ESC command normalized into [-8192, 8191]
#define UAVCAN_ESC_MAX_VALUE    8191

#define SERVO_OUT_RCIN_MAX      16  // SRV_Channel::k_rcin1 ... SRV_Channel::k_rcin16
#define SERVO_OUT_MOTOR_MAX     12  // SRV_Channel::k_motor1 ... SRV_Channel::k_motor8, SRV_Channel::k_motor9 ... SRV_Channel::k_motor12

extern const AP_HAL::HAL &hal;

void AP_Periph_FW::rcout_init()
{
    // start up with safety enabled. This disables the pwm output until we receive an packet from the rempte system
    hal.rcout->force_safety_on();

#if HAL_PWM_COUNT > 0
    for (uint8_t i=0; i<HAL_PWM_COUNT; i++) {
        servo_channels.set_default_function(i, SRV_Channel::Aux_servo_function_t(SRV_Channel::k_rcin1 + i));
    }
#endif

    for (uint8_t i=0; i<SERVO_OUT_RCIN_MAX; i++) {
        SRV_Channels::set_angle(SRV_Channel::Aux_servo_function_t(SRV_Channel::k_rcin1 + i), 1000);
    }

    uint16_t esc_mask = 0;
    for (uint8_t i=0; i<SERVO_OUT_MOTOR_MAX; i++) {
        SRV_Channels::set_range(SRV_Channels::get_motor_function(i), UAVCAN_ESC_MAX_VALUE);
        uint8_t chan;
        if (SRV_Channels::find_channel(SRV_Channels::get_motor_function(i), chan)) {
            esc_mask |= 1U << chan;
        }
    }

    // setup ESCs with the desired PWM type, allowing for DShot
    hal.rcout->set_output_mode(esc_mask, (AP_HAL::RCOutput::output_mode)g.esc_pwm_type.get());

    // run this once and at 1Hz to configure aux and esc ranges
    rcout_init_1Hz();

    // run DShot at 1kHz
    hal.rcout->set_dshot_rate(0, 400);
}

void AP_Periph_FW::rcout_init_1Hz()
{
    // this runs at 1Hz to allow for run-time param changes
    SRV_Channels::enable_aux_servos();

    for (uint8_t i=0; i<SERVO_OUT_MOTOR_MAX; i++) {
        servo_channels.set_esc_scaling_for(SRV_Channels::get_motor_function(i));
    }
}

void AP_Periph_FW::rcout_esc(int16_t *rc, uint8_t num_channels)
{
    if (rc == nullptr) {
        return;
    }

    const uint8_t channel_count = MIN(num_channels, SERVO_OUT_MOTOR_MAX);
    for (uint8_t i=0; i<channel_count; i++) {
        // we don't support motor reversal yet on ESCs in AP_Periph
        SRV_Channels::set_output_scaled(SRV_Channels::get_motor_function(i), MAX(0,rc[i]));
    }

    rcout_has_new_data_to_update = true;
}

void AP_Periph_FW::rcout_srv(uint8_t actuator_id, const float command_value)
{
#if HAL_PWM_COUNT > 0
    if ((actuator_id == 0) || (actuator_id > HAL_PWM_COUNT)) {
        // not supported or out of range
        return;
    }

    const SRV_Channel::Aux_servo_function_t function = SRV_Channel::Aux_servo_function_t(SRV_Channel::k_rcin1 + actuator_id - 1);
    SRV_Channels::set_output_norm(function, command_value);

    rcout_has_new_data_to_update = true;
#endif
}

void AP_Periph_FW::rcout_handle_safety_state(uint8_t safety_state)
{
    if (safety_state == 255) {
        hal.rcout->force_safety_off();
    } else {
        hal.rcout->force_safety_on();
    }
    rcout_has_new_data_to_update = true;
}

void AP_Periph_FW::rcout_update()
{
    if (!rcout_has_new_data_to_update) {
        return;
    }
    rcout_has_new_data_to_update = false;

    SRV_Channels::calc_pwm();
    SRV_Channels::cork();
    SRV_Channels::output_ch_all();
    SRV_Channels::push();
}

#endif // HAL_PERIPH_ENABLE_RC_OUT

