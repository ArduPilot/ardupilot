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

// table of user settable parameters
const AP_Param::GroupInfo IntelligentEnergy24::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: IntelligentEnergy 2.4kWh FuelCell sim enable/disable
    // @Description: Allows you to enable (1) or disable (0) the FuelCell simulator
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("ENABLE", 1, IntelligentEnergy24, enabled, 0),

    // @Param: STATE
    // @DisplayName: Explicitly set state
    // @Description: Explicity specify a state for the generator to be in
    // @User: Advanced
    AP_GROUPINFO("STATE", 2, IntelligentEnergy24, set_state, -1),

    // @Param: ERROR
    // @DisplayName: Explicitly set error code
    // @Description: Explicity specify an error code to send to the generator
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
    // gcs().send_text(MAV_SEVERITY_INFO, "fuelcell update");
    update_send();
}

void IntelligentEnergy24::update_send()
{
    // just send a chunk of data at 1Hz:
    const uint32_t now = AP_HAL::millis();
    if (now - last_sent_ms < 500) {
        return;
    }

    // Simulate constant current charge/discharge of the battery
    float amps = discharge ? -20.0f : 20.0f;

    // Simulate constant tank pressure. This isn't true in reality, but is good enough
    const int16_t tank_bar = 250;

    // Update pack capacity remaining
    bat_capacity_mAh += amps*(now - last_sent_ms)/3600.0f;

    // From capacity remaining approximate voltage by linear interpolation
    const float min_bat_vol = 42.0f;
    const float max_bat_vol = 50.4f;
    const float max_bat_capactiy_mAh = 3300;

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

    last_sent_ms = now;

    char message[128];
    hal.util->snprintf(message, ARRAY_SIZE(message), "<%i,%.1f,%i,%u,%i,%u,%u>\n",
             tank_bar,
             battery_voltage,
             (signed)pwr_out,
             (unsigned)spm_pwr,
             (signed)battery_pwr,
             (unsigned)state,
             (unsigned)err_code);

    if ((unsigned)write_to_autopilot(message, strlen(message)) != strlen(message)) {
        AP_HAL::panic("Failed to write to autopilot: %s", strerror(errno));
    }
}
