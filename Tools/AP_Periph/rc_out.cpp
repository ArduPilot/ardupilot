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

#define SERVO_OUT_RCIN_MAX      32  // note that we allow for more than is in the enum
#define SERVO_OUT_MOTOR_MAX     12  // SRV_Channel::k_motor1 ... SRV_Channel::k_motor8, SRV_Channel::k_motor9 ... SRV_Channel::k_motor12

extern const AP_HAL::HAL &hal;

void AP_Periph_FW::rcout_init()
{
    // start up with safety enabled. This disables the pwm output until we receive an packet from the rempte system
    hal.rcout->force_safety_on();

#if HAL_WITH_ESC_TELEM && !HAL_GCS_ENABLED
    if (g.esc_telem_port >= 0) {
        serial_manager.set_protocol_and_baud(g.esc_telem_port, AP_SerialManager::SerialProtocol_ESCTelemetry, 115200);
    }
#endif

#if HAL_PWM_COUNT > 0
    for (uint8_t i=0; i<HAL_PWM_COUNT; i++) {
        servo_channels.set_default_function(i, SRV_Channel::Aux_servo_function_t(SRV_Channel::k_rcin1 + i));
    }
#endif

    for (uint8_t i=0; i<SERVO_OUT_RCIN_MAX; i++) {
        SRV_Channels::set_angle(SRV_Channel::Aux_servo_function_t(SRV_Channel::k_rcin1 + i), 1000);
    }

    uint32_t esc_mask = 0;
    for (uint8_t i=0; i<SERVO_OUT_MOTOR_MAX; i++) {
        SRV_Channels::set_range(SRV_Channels::get_motor_function(i), UAVCAN_ESC_MAX_VALUE);
        uint8_t chan;
        if (SRV_Channels::find_channel(SRV_Channels::get_motor_function(i), chan)) {
            esc_mask |= 1U << chan;
        }
    }

    // run this once and at 1Hz to configure aux and esc ranges
    rcout_init_1Hz();

    // setup ESCs with the desired PWM type, allowing for DShot
    SRV_Channels::init(esc_mask, (AP_HAL::RCOutput::output_mode)g.esc_pwm_type.get());

    // run DShot at 1kHz
    hal.rcout->set_dshot_rate(SRV_Channels::get_dshot_rate(), 400);
#if HAL_WITH_ESC_TELEM
    esc_telem_update_period_ms = 1000 / constrain_int32(g.esc_telem_rate.get(), 1, 1000);
#endif
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

void AP_Periph_FW::rcout_srv_unitless(uint8_t actuator_id, const float command_value)
{
#if HAL_PWM_COUNT > 0
    const SRV_Channel::Aux_servo_function_t function = SRV_Channel::Aux_servo_function_t(SRV_Channel::k_rcin1 + actuator_id - 1);
    SRV_Channels::set_output_norm(function, command_value);

    rcout_has_new_data_to_update = true;
#endif
}

void AP_Periph_FW::rcout_srv_PWM(uint8_t actuator_id, const float command_value)
{
#if HAL_PWM_COUNT > 0
    const SRV_Channel::Aux_servo_function_t function = SRV_Channel::Aux_servo_function_t(SRV_Channel::k_rcin1 + actuator_id - 1);
    SRV_Channels::set_output_pwm(function, uint16_t(command_value+0.5));

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
    uint32_t now_ms = AP_HAL::millis();

    const uint16_t esc_timeout_ms = g.esc_command_timeout_ms >= 0 ? g.esc_command_timeout_ms : 0; // Don't allow negative timeouts!
    const bool has_esc_rawcommand_timed_out = esc_timeout_ms != 0 && ((now_ms - last_esc_raw_command_ms) >= esc_timeout_ms);
    if (last_esc_num_channels > 0 && has_esc_rawcommand_timed_out) {
        // If we've seen ESCs previously, and a timeout has occurred, then zero the outputs
        int16_t esc_output[last_esc_num_channels] {};
        rcout_esc(esc_output, last_esc_num_channels);

        // register that the output has been changed
        rcout_has_new_data_to_update = true;
    }

    if (!rcout_has_new_data_to_update) {
        return;
    }
    rcout_has_new_data_to_update = false;

    SRV_Channels::calc_pwm();
    SRV_Channels::cork();
    SRV_Channels::output_ch_all();
    SRV_Channels::push();
#if HAL_WITH_ESC_TELEM
    if (now_ms - last_esc_telem_update_ms >= esc_telem_update_period_ms) {
        last_esc_telem_update_ms = now_ms;
        esc_telem_update();
    }
#endif
}

#endif // HAL_PERIPH_ENABLE_RC_OUT
