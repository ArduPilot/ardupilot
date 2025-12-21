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
#if AP_PERIPH_RC_OUT_ENABLED
#include "AP_Periph.h"
#if AP_SIM_ENABLED
#include <dronecan_msgs.h>
#endif

// magic value from UAVCAN driver packet
// dsdl/uavcan/equipment/esc/1030.RawCommand.uavcan
// Raw ESC command normalized into [-8192, 8191]
#define UAVCAN_ESC_MAX_VALUE    8191

#define SERVO_OUT_RCIN_MAX      32  // note that we allow for more than is in the enum
#ifndef SERVO_OUT_MOTOR_MAX
#define SERVO_OUT_MOTOR_MAX     32  // SRV_Channel::k_motor1 ... SRV_Channel::k_motor8, SRV_Channel::k_motor9 ... SRV_Channel::k_motor12, SRV_Channel::k_motor13 ... SRV_Channel::k_motor32
#endif

extern const AP_HAL::HAL &hal;

void AP_Periph_FW::rcout_init()
{
#if AP_PERIPH_SAFETY_SWITCH_ENABLED
    // start up with safety enabled. This disables the pwm output until we receive an packet from the rempte system
    hal.rcout->force_safety_on();
#else
    hal.rcout->force_safety_off();
#endif

#if HAL_WITH_ESC_TELEM && !HAL_GCS_ENABLED
    if (g.esc_telem_port >= 0) {
        serial_manager.set_protocol_and_baud(g.esc_telem_port, AP_SerialManager::SerialProtocol_ESCTelemetry, 115200);
    }
#endif

#if HAL_PWM_COUNT > 0
    for (uint8_t i=0; i<HAL_PWM_COUNT; i++) {
        servo_channels.set_default_function(i, SRV_Channel::Function(SRV_Channel::k_rcin1 + i));
    }
#endif

    for (uint8_t i=0; i<SERVO_OUT_RCIN_MAX; i++) {
        SRV_Channels::set_angle(SRV_Channel::Function(SRV_Channel::k_rcin1 + i), 1000);
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

#if HAL_DSHOT_ENABLED
    hal.rcout->set_dshot_esc_type(SRV_Channels::get_dshot_esc_type());
#endif

    // run PWM ESCs at configured rate
    hal.rcout->set_freq(esc_mask, g.esc_rate.get());

    // setup ESCs with the desired PWM type, allowing for DShot
    AP::srv().init(esc_mask, (AP_HAL::RCOutput::output_mode)g.esc_pwm_type.get());

    // run DShot at 1kHz
    hal.rcout->set_dshot_rate(SRV_Channels::get_dshot_rate(), 400);
#if HAL_WITH_ESC_TELEM
    esc_telem_update_period_ms = 1000 / constrain_int32(g.esc_telem_rate.get(), 1, 1000);
#endif
}

void AP_Periph_FW::rcout_init_1Hz()
{
    // this runs at 1Hz to allow for run-time param changes
    AP::srv().enable_aux_servos();

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
    const SRV_Channel::Function function = SRV_Channel::Function(SRV_Channel::k_rcin1 + actuator_id - 1);
    SRV_Channels::set_output_norm(function, command_value);

    // Add to mask of channels that will be cleared if no commands are received
    actuator.mask |= SRV_Channels::get_output_channel_mask(function);

    rcout_has_new_data_to_update = true;
#if AP_SIM_ENABLED
    sim_update_actuator(actuator_id);
#endif
#endif
}

void AP_Periph_FW::rcout_srv_PWM(uint8_t actuator_id, const float command_value)
{
#if HAL_PWM_COUNT > 0
    const SRV_Channel::Function function = SRV_Channel::Function(SRV_Channel::k_rcin1 + actuator_id - 1);
    SRV_Channels::set_output_pwm(function, uint16_t(command_value+0.5));

    // Add to mask of channels that will be cleared if no commands are received
    actuator.mask |= SRV_Channels::get_output_channel_mask(function);

    rcout_has_new_data_to_update = true;
#if AP_SIM_ENABLED
    sim_update_actuator(actuator_id);
#endif
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

    // Timeout for ESC commands
    const uint16_t esc_timeout_ms = g.esc_command_timeout_ms >= 0 ? g.esc_command_timeout_ms : 0; // Don't allow negative timeouts!
    const bool has_esc_rawcommand_timed_out = esc_timeout_ms != 0 && ((now_ms - last_esc_raw_command_ms) >= esc_timeout_ms);
    if (last_esc_num_channels > 0 && has_esc_rawcommand_timed_out) {
        // If we've seen ESCs previously, and a timeout has occurred, then zero the outputs
        int16_t esc_output[last_esc_num_channels];
        memset(esc_output, 0, sizeof(esc_output));
        rcout_esc(esc_output, last_esc_num_channels);

        // Don't need to run again until new commands have been received
        last_esc_num_channels = 0;
    }

    // Timeout for servo actuator commands
    const uint16_t servo_timeout_ms = g.servo_command_timeout_ms >= 0 ? g.servo_command_timeout_ms : 0; // Don't allow negative timeouts!
    const bool has_servo_timed_out = servo_timeout_ms != 0 && ((now_ms - actuator.last_command_ms) >= servo_timeout_ms);
    if (has_servo_timed_out && (actuator.mask != 0)) {
#if HAL_PWM_COUNT > 0
        // Output 0 PWM for each channel in the mask
        for (uint8_t i = 0; i < HAL_PWM_COUNT; i++) {
            if (((1U<<i) & actuator.mask) != 0) {
                SRV_Channels::set_output_pwm_chan(i, 0);
            }
        }

        // register that the output has been changed
        rcout_has_new_data_to_update = true;
#endif

        // Don't need to run again until new commands have been received
        actuator.mask = 0;
    }

    if (!rcout_has_new_data_to_update) {
        return;
    }
    rcout_has_new_data_to_update = false;

    auto &srv = AP::srv();
    SRV_Channels::calc_pwm();
    srv.cork();
    SRV_Channels::output_ch_all();
    srv.push();
#if HAL_WITH_ESC_TELEM
    if (now_ms - last_esc_telem_update_ms >= esc_telem_update_period_ms) {
        last_esc_telem_update_ms = now_ms;
        esc_telem_update();
    }
#if AP_EXTENDED_ESC_TELEM_ENABLED
    esc_telem_extended_update(now_ms);
#endif
#endif
}

#if AP_SIM_ENABLED
/*
  update simulation of servos, sending actuator status
*/
void AP_Periph_FW::sim_update_actuator(uint8_t actuator_id)
{
    sim_actuator.mask |= 1U << (actuator_id - 1);

    // send status at 10Hz
    const uint32_t period_ms = 100;
    const uint32_t now_ms = AP_HAL::millis();

    if (now_ms - sim_actuator.last_send_ms < period_ms) {
        return;
    }
    sim_actuator.last_send_ms = now_ms;

    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        if ((sim_actuator.mask & (1U<<i)) == 0) {
            continue;
        }
        const SRV_Channel::Function function = SRV_Channel::Function(SRV_Channel::k_rcin1 + i);
        uavcan_equipment_actuator_Status pkt {};
        pkt.actuator_id = i + 1;
        // assume 45 degree angle for simulation
        pkt.position = radians(SRV_Channels::get_output_norm(function) * 45);
        pkt.force = 0;
        pkt.speed = 0;
        pkt.power_rating_pct = UAVCAN_EQUIPMENT_ACTUATOR_STATUS_POWER_RATING_PCT_UNKNOWN;

        uint8_t buffer[UAVCAN_EQUIPMENT_ACTUATOR_STATUS_MAX_SIZE];
        uint16_t total_size = uavcan_equipment_actuator_Status_encode(&pkt, buffer, !canfdout());

        canard_broadcast(UAVCAN_EQUIPMENT_ACTUATOR_STATUS_SIGNATURE,
                         UAVCAN_EQUIPMENT_ACTUATOR_STATUS_ID,
                         CANARD_TRANSFER_PRIORITY_LOW,
                         &buffer[0],
                         total_size);
    }
}
#endif // AP_SIM_ENABLED

#endif // AP_PERIPH_RC_OUT_ENABLED
