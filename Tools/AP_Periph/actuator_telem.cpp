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

// ActuatorChannel parameter group
const AP_Param::GroupInfo ActuatorChannel::var_info[] = {
    // @Param: _CURR_PIN
    // @DisplayName: Current sensing pin
    // @Description: Analog input pin number for current sensing. Set to -1 to disable.
    // @Values: -1:Disabled
    // @Range: -1 15
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("CURR_PIN", 1, ActuatorChannel, curr_pin, AP_PERIPH_ACTUATOR_TELEM_CURR_PIN_DEFAULT),

    // @Param: _AMP_OFFSET
    // @DisplayName: Current sensor offset
    // @Description: Voltage offset at zero current on the current sensor.
    // @Units: V
    // @Range: -10 10
    // @User: Standard
    AP_GROUPINFO("AMP_OFFSET", 2, ActuatorChannel, curr_amp_offset, AP_PERIPH_ACTUATOR_TELEM_CURR_AMP_OFFSET_DEFAULT),

    // @Param: _AMP_PERVLT
    // @DisplayName: Amps per volt
    // @Description: Current sensor scale factor.
    // @Units: A/V
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("AMP_PERVLT", 3, ActuatorChannel, curr_amp_per_volt, AP_PERIPH_ACTUATOR_TELEM_CURR_AMP_PERVLT_DEFAULT),

    // @Param: _CURR_MAX
    // @DisplayName: Maximum current
    // @Description: Maximum expected current for this channel.
    // @Units: A
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("CURR_MAX", 4, ActuatorChannel, curr_max, AP_PERIPH_ACTUATOR_TELEM_CURR_MAX_DEFAULT),

    AP_GROUPEND
};

const AP_Param::GroupInfo ActuatorTelem::var_info[] = {
    // @Param: _RATE
    // @DisplayName: Actuator Telemetry rate
    // @Description: Actuator Telemetry update rate in Hz. Set to 0 to disable.
    // @Units: Hz
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("_TELEM_RATE", 1, ActuatorTelem, rate, AP_PERIPH_ACTUATOR_TELEM_RATE_DEFAULT),

#if (HAL_ACTUATOR_TELEM_CHANNELS >= 1)
    // @Group: 1_
    // @Path: actuator_telem.cpp
    AP_SUBGROUPINFO(channels[0], "1_", 2, ActuatorTelem, ActuatorChannel),
#endif

#if (HAL_ACTUATOR_TELEM_CHANNELS >= 2)
    // @Group: 2_
    // @Path: actuator_telem.cpp
    AP_SUBGROUPINFO(channels[1], "2_", 3, ActuatorTelem, ActuatorChannel),
#endif

#if (HAL_ACTUATOR_TELEM_CHANNELS >= 3)
    // @Group: 3_
    // @Path: actuator_telem.cpp
    AP_SUBGROUPINFO(channels[2], "3_", 4, ActuatorTelem, ActuatorChannel),
#endif

#if (HAL_ACTUATOR_TELEM_CHANNELS >= 4)
    // @Group: 4_
    // @Path: actuator_telem.cpp
    AP_SUBGROUPINFO(channels[3], "4_", 5, ActuatorTelem, ActuatorChannel),
#endif

    AP_GROUPEND
};

ActuatorChannel::ActuatorChannel(void) :
    analog_source(nullptr)
{
}

void ActuatorChannel::init(void)
{
    if (curr_pin >= 0 && analog_source == nullptr) {
        analog_source = hal.analogin->channel(curr_pin);
    }
}

void ActuatorChannel::send_telemetry(uint8_t actuator_id)
{
    // Check if this channel is enabled
    if (curr_pin < 0 || analog_source == nullptr) {
        return;
    }

    // Read current from analog input
    float current_amp = 0;
    float adc_voltage = analog_source->voltage_average();
    current_amp = (adc_voltage - curr_amp_offset) * curr_amp_per_volt;

    uavcan_equipment_actuator_Status pkt {}; 

    pkt.actuator_id = actuator_id;
    pkt.position = nanf(""); // Not available
    pkt.force = nanf("");    // Not available
    pkt.speed = nanf("");    // Not available

    // Calculate power rating percentage from current
    const float max_current = curr_max;
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

ActuatorTelem::ActuatorTelem(void)
{
    AP_Param::setup_object_defaults(this, var_info);
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
    for (uint8_t i = 0; i < HAL_PWM_COUNT; i++) {
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

        // Initialize channel if needed
        channels[i].init();

        channels[i].send_telemetry(actuator_id);
    }
#endif
}

#endif  // AP_PERIPH_ACTUATOR_TELEM_ENABLED
