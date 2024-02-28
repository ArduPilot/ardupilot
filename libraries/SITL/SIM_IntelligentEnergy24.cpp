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
  Simulator for the IntelligentEnergy 2.4kWh FuelCell generator
*/

#include <AP_Math/AP_Math.h>

#include "SIM_IntelligentEnergy24.h"
#include "SITL.h"

#include <errno.h>

#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

using namespace SITL;

#define MAX_TANK_PRESSURE 300 //(bar)

// table of user settable parameters
const AP_Param::GroupInfo IntelligentEnergy24::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: IntelligentEnergy 2.4kWh FuelCell sim enable/disable
    // @Description: Allows you to enable (1) or disable (0) the FuelCell simulator
    // @Values: 0:Disabled,1:V1 Protocol,2:V2 Protocol
    // @User: Advanced
    AP_GROUPINFO("ENABLE", 1, IntelligentEnergy24, enabled, 0),

    // @Param: STATE
    // @DisplayName: Explicitly set state
    // @Description: Explicitly specify a state for the generator to be in
    // @User: Advanced
    AP_GROUPINFO("STATE", 2, IntelligentEnergy24, set_state, -1),

    // @Param: ERROR
    // @DisplayName: Explicitly set error code
    // @Description: Explicitly specify an error code to send to the generator
    // @User: Advanced
    AP_GROUPINFO("ERROR", 3, IntelligentEnergy24, err_code, 0),

    AP_GROUPEND
};

IntelligentEnergy24::IntelligentEnergy24() : IntelligentEnergy::IntelligentEnergy()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void IntelligentEnergy24::update(const struct sitl_input &input)
{
    if (!enabled.get()) {
        return;
    }
    update_send();
}

void IntelligentEnergy24::update_send()
{
    // just send a chunk of data at 2 Hz:
    const uint32_t now = AP_HAL::millis();
    if (now - last_data_sent_ms < 500) {
        return;
    }

    // Simulate constant current charge/discharge of the battery
    float amps = discharge ? -20.0f : 20.0f;

    // Update pack capacity remaining
    bat_capacity_mAh += amps*(now - last_data_sent_ms)/3600.0f;

    // From capacity remaining approximate voltage by linear interpolation
    const float min_bat_vol = 42.0f;
    const float max_bat_vol = 50.4f;
    const float max_bat_capactiy_mAh = 3300;

    // Simulate tank pressure
    // Scale tank pressure linearly to a percentage.
    // Min = 5 bar, max = 300 bar, PRESS_GRAD = 1/295.
    const int16_t tank_bar = linear_interpolate(5, MAX_TANK_PRESSURE, bat_capacity_mAh / max_bat_capactiy_mAh, 0, 1);

    battery_voltage = bat_capacity_mAh / max_bat_capactiy_mAh * (max_bat_vol - min_bat_vol) + min_bat_vol;

    // Decide if we need to charge or discharge the battery
    if (battery_voltage <= min_bat_vol) {
        discharge = false;
    } else if (battery_voltage >= max_bat_vol) {
        discharge = true;
    }

    int32_t battery_pwr = battery_voltage * amps; // Watts

    // These are non-physical values
    const int32_t pwr_out = float_to_int32(battery_pwr*1.4f);
    const uint32_t spm_pwr = float_to_uint32(battery_pwr*0.3f);

    uint32_t state = set_state;
    if (set_state == -1) {
        state = 2; // Running
    }

    last_data_sent_ms = now;

    char message[128];

    if (enabled.get() == 1) {
        // V1 Protocol
        hal.util->snprintf(message, ARRAY_SIZE(message), "<%i,%.1f,%i,%u,%i,%u,%u>\n",
             tank_bar,
             battery_voltage,
             (signed)pwr_out,
             (unsigned)spm_pwr,
             (signed)battery_pwr,
             (unsigned)state,
             (unsigned)err_code);

    } else {
        // V2 Protocol

        // version message sent at 0.2 Hz
        if (now - last_ver_sent_ms > 5e3) {
            // PCM software part number, software version number, protocol number, hardware serial number, check-sum
            hal.util->snprintf(message, ARRAY_SIZE(message), "[10011867,2.132,4,IE12160A8040015,7]\n");

            if ((unsigned)write_to_autopilot(message, strlen(message)) != strlen(message)) {
                AP_HAL::panic("Failed to write to autopilot: %s", strerror(errno));
            }
            last_ver_sent_ms = now;
        }

        // data message
        memset(&message, 0, sizeof(message));
        int8_t tank_remaining_pct = (float)tank_bar / MAX_TANK_PRESSURE * 100.0;

        hal.util->snprintf(message, ARRAY_SIZE(message), "<%i,%.2f,%.1f,%i,%u,%i,%i,%u,%u,%i,%s,", // last blank , is for fuel cell to send info string up to 32 char ASCII
             tank_remaining_pct,
             0.67f, // inlet pressure (bar)
             battery_voltage,
             (signed)pwr_out,
             (unsigned)spm_pwr,
             0, // unit at fault (0 = no fault)
             (signed)battery_pwr,
             (unsigned)state,
             (unsigned)err_code,
             0, // fault state 2 (0 = no fault)
             get_error_string(err_code)); 

        // calculate the checksum
        uint8_t checksum = 0;
        for (uint8_t i = 0; i < ARRAY_SIZE(message); i++) {
            if (message[i] == 0) {
                break;
            }
            checksum += message[i];
        }
        // checksum is inverted 8-bit
        checksum = ~checksum;

        // add the checksum to the end of the message
        char data_end[7];
        hal.util->snprintf(data_end, ARRAY_SIZE(data_end), "%u>\n", checksum);
        strncat(message, data_end, ARRAY_SIZE(data_end));

    }

    if ((unsigned)write_to_autopilot(message, strlen(message)) != strlen(message)) {
        AP_HAL::panic("Failed to write to autopilot: %s", strerror(errno));
    }
}

const char * IntelligentEnergy24::get_error_string(const uint32_t code)
{
    switch (code) {
        case 20:
            return "THERMAL MNGMT";

        default:
            break;
    }

    return "";
}
