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

#include "AP_Periph.h"

#if AP_PERIPH_ACTUATOR_TELEM_ENABLED

#include <dronecan_msgs.h>

extern const AP_HAL::HAL &hal;

#ifndef AP_PERIPH_ACTUATOR_TELEM_RATE_DEFAULT
#define AP_PERIPH_ACTUATOR_TELEM_RATE_DEFAULT 10
#endif

#ifndef AP_PERIPH_ACTUATOR_TELEM_NUM_CHANNELS_DEFAULT
#define AP_PERIPH_ACTUATOR_TELEM_NUM_CHANNELS_DEFAULT 0
#endif

#ifndef AP_PERIPH_ACTUATOR_TELEM_CURR_PIN_DEFAULT
#define AP_PERIPH_ACTUATOR_TELEM_CURR_PIN_DEFAULT -1
#endif

#ifndef AP_PERIPH_ACTUATOR_TELEM_CURR_AMP_OFFSET_DEFAULT
#define AP_PERIPH_ACTUATOR_TELEM_CURR_AMP_OFFSET_DEFAULT 0.0f
#endif

#ifndef AP_PERIPH_ACTUATOR_TELEM_CURR_AMP_PERVLT_DEFAULT
#define AP_PERIPH_ACTUATOR_TELEM_CURR_AMP_PERVLT_DEFAULT 10
#endif

#ifndef AP_PERIPH_ACTUATOR_TELEM_CURR_MAX_DEFAULT
#define AP_PERIPH_ACTUATOR_TELEM_CURR_MAX_DEFAULT 2.5f
#endif

const AP_Param::GroupInfo ActuatorTelem::var_info[] = {
    // @Param: _TELEM_RATE
    // @DisplayName: Actuator Telemetry rate
    // @Description: Actuator Telemetry update rate in Hz. Set to 0 to disable.
    // @Units: Hz
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("_TELEM_RATE", 1, ActuatorTelem, rate, AP_PERIPH_ACTUATOR_TELEM_RATE_DEFAULT),

    // @Param: _NUM_CHANS
    // @DisplayName: Number of actuator channels
    // @Description: Number of actuator channels to monitor for telemetry.
    // @Range: 0 4 
    // @User: Standard
    AP_GROUPINFO("_NUM_CHANS", 2, ActuatorTelem, num_chans, AP_PERIPH_ACTUATOR_TELEM_NUM_CHANNELS_DEFAULT),

    // @Param: _CURR_PIN1
    // @DisplayName: Current sensing pin 1
    // @Description: Analog input pin number for current sensing on channel 1. Set to -1 to disable.
    // @Values: -1:Disabled
    // @Range: -1 127
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("_CURR_PIN1", 3, ActuatorTelem, curr_pin1, AP_PERIPH_ACTUATOR_TELEM_CURR_PIN_DEFAULT),

    // @Param: _AMP_OFFSET
    // @DisplayName: Current sensor offset
    // @Description: Voltage offset at zero current on the current sensor.
    // @Units: V
    // @User: Standard
    AP_GROUPINFO("AMP_OFFSET", 4, ActuatorTelem, curr_amp_offset, AP_PERIPH_ACTUATOR_TELEM_CURR_AMP_OFFSET_DEFAULT),

    // @Param: _AMP_PERVLT
    // @DisplayName: Amps per volt
    // @Description: Current sensor scale factor.
    // @Units: A/V
    // @User: Standard
    AP_GROUPINFO("AMP_PERVLT", 5, ActuatorTelem, curr_amp_per_volt, AP_PERIPH_ACTUATOR_TELEM_CURR_AMP_PERVLT_DEFAULT),

    // @Param: _CURR_MAX
    // @DisplayName: Maximum current
    // @Description: Maximum expected current for this channel.
    // @Units: A
    // @User: Standard
    AP_GROUPINFO("CURR_MAX", 6, ActuatorTelem, curr_max, AP_PERIPH_ACTUATOR_TELEM_CURR_MAX_DEFAULT),

    AP_GROUPEND
};

ActuatorTelem::ActuatorTelem(void)
{
    AP_Param::setup_object_defaults(this, var_info);
    for (uint8_t i = 0; i < HAL_ACTUATOR_TELEM_CURR_MAX_CHANNELS; i++) {
        analog_sources[i] = nullptr;
    }
}

void ActuatorTelem::init(void)
{
    const int8_t curr_pin1_val = curr_pin1.get();
    const int8_t num_chans_val = num_chans.get();
    
    if (curr_pin1_val >= 0 && num_chans_val > 0) {
        for (uint8_t i = 0; i < MIN(HAL_ACTUATOR_TELEM_CURR_MAX_CHANNELS, num_chans_val); i++) {
            const int8_t pin = curr_pin1_val + i;
            if (pin >= 0) {
                analog_sources[i] = hal.analogin->channel(pin);
            }
        }
    }
}

void ActuatorTelem::send_telemetry(uint8_t channel_index, uint8_t actuator_id)
{
    // Check if this channel is enabled
    if (channel_index >= HAL_ACTUATOR_TELEM_CURR_MAX_CHANNELS) {
        return;
    }

    AP_HAL::AnalogSource *source = analog_sources[channel_index];
    if (source == nullptr) {
        return;
    }
    // Read current from analog input
    float current_amp = 0;
    float adc_voltage = source->voltage_average();
    current_amp = (adc_voltage - curr_amp_offset) * curr_amp_per_volt;

    uavcan_equipment_actuator_Status pkt {}; 

    pkt.actuator_id = actuator_id;
    pkt.position = nanf(""); // Not available
    pkt.force = nanf("");    // Not available
    pkt.speed = nanf("");    // Not available

    // Calculate power rating percentage from current
    const float max_current = curr_max.get();
    if (max_current > 0 && current_amp >= 0) {
        pkt.power_rating_pct = constrain_int16(current_amp / max_current * 100.0f, 0, 100);
    } else {
        pkt.power_rating_pct = UAVCAN_EQUIPMENT_ACTUATOR_STATUS_POWER_RATING_PCT_UNKNOWN;
    }

    // encode and broadcast
    uint8_t buffer[UAVCAN_EQUIPMENT_ACTUATOR_STATUS_MAX_SIZE];
    uint16_t total_size = uavcan_equipment_actuator_Status_encode(&pkt, buffer, !periph.canfdout());

    periph.canard_broadcast(UAVCAN_EQUIPMENT_ACTUATOR_STATUS_SIGNATURE,
                            UAVCAN_EQUIPMENT_ACTUATOR_STATUS_ID,
                            CANARD_TRANSFER_PRIORITY_LOW,
                            &buffer[0],
                            total_size);
}

void ActuatorTelem::update()
{
    // Check global telemetry rate
    const float telem_rate = rate.get();
    if (telem_rate <= 0) {
        return;  // telemetry disabled
    }

    const uint32_t now = AP_HAL::millis();
    if (now - last_telem_update_ms < 1000.0f / telem_rate) {
        return;
    }
    last_telem_update_ms = now;

#if HAL_PWM_COUNT > 0
    const int8_t num_chans_val = num_chans.get();
    if (num_chans_val > 0) {
        for (uint8_t i = 0; i < MIN(HAL_PWM_COUNT, num_chans_val); i++) {
            const auto *srv_channel = periph.servo_channels.srv_channel(i);
            if (srv_channel == nullptr) {
                continue;
            }

            const SRV_Channel::Function function = srv_channel->get_function();
            // Only send for configured actuator functions
            if (function < SRV_Channel::k_rcin1 || function > SRV_Channel::k_rcin16) {
                continue;
            }

            const uint8_t actuator_id = function - SRV_Channel::k_rcin1 + 1;

            send_telemetry(i, actuator_id);
        }
    }
#endif
}

#endif  // AP_PERIPH_ACTUATOR_TELEM_ENABLED
