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

#include "AP_BattMonitor_config.h"

#if AP_BATTERY_GENERATOR_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>

#include "AP_BattMonitor_Generator.h"

/*
    Fuel class
*/
// This is where we tell the battery monitor 'we have current' if we want to report a fuel level remaining
bool AP_BattMonitor_Generator_FuelLevel::has_current(void) const
{
    // If the generator has fuel remaining we must also state that we have current
    return has_consumed_energy();
}

// This is where we tell the battery monitor 'we have consummed energy' if we want to report a fuel level remaining
bool AP_BattMonitor_Generator_FuelLevel::has_consumed_energy(void) const
{
    // Get pointer to generator singleton
    AP_Generator *generator = AP::generator();

    if (generator == nullptr) {
        return false;
    }

    // Use consumed_mAh in BattMonitor to display fuel remaining
    return generator->has_fuel_remaining();
}

void AP_BattMonitor_Generator_FuelLevel::init()
{
    // Set params for users:
    // Fuel level is only reported as a percentage
    _params._pack_capacity.set(100);
    // Fuel only reports a fixed 1v, don't want batt monitor failsafes on this instance
    _params._low_voltage.set(0);
    _params._critical_voltage.set(0);
}

// Read the fuel level.  Should be called at 10hz
void AP_BattMonitor_Generator_FuelLevel::read()
{
    _state.healthy = false;

    // Get pointer to generator singleton
    AP_Generator *generator = AP::generator();

    // Not healthy if we can't find a generator
    if (generator == nullptr) {
        return;
    }

    if (!generator->healthy()) {
        return;
    }

    // As this is a battery monitor instance report voltage
    // Report fixed voltage of 1V
    _state.voltage = 1.0f;

    // This is a bodge to display tank level as a percentage on GCS.  Users should set _params.pack_capacity == 100 to get a clear percentage in GCS
    _state.consumed_mah = (1 - generator->get_fuel_remaining()) * _params._pack_capacity.get();

    // If we got this far then must be healthy
    _state.healthy = true;
    _state.last_time_micros = AP_HAL::micros();
}

/*
    Electrical class
*/
bool AP_BattMonitor_Generator_Elec::has_current(void) const
{
    // Get pointer to generator singleton
    AP_Generator *generator = AP::generator();

    if (generator == nullptr) {
        return false;
    }

    return generator->has_current();
}

bool AP_BattMonitor_Generator_Elec::has_consumed_energy(void) const
{
    // Get pointer to generator singleton
    AP_Generator *generator = AP::generator();

    if (generator == nullptr) {
        return false;
    }

    return generator->has_consumed_energy();
}

// Read the electrical measurements from the generator
void AP_BattMonitor_Generator_Elec::read()
{
    _state.healthy = false;

    // Get pointer to generator singleton
    AP_Generator *generator = AP::generator();

    // Not healthy if we can't find a generator
    if (generator == nullptr) {
        return;
    }

    if (!generator->healthy()) {
        return;
    }

    // Update readings
    _state.voltage = generator->get_voltage();

    _state.current_amps = generator->get_current();

    // Always reset consumed value, integration is done in AP_Generator library
    _state.consumed_mah = generator->get_batt_consumed();
    _state.consumed_wh = 0.001f * _state.consumed_mah * _state.voltage;

    // If we got this far then must be healthy
    _state.healthy = true;
    _state.last_time_micros = AP_HAL::micros();
}

AP_BattMonitor::Failsafe AP_BattMonitor_Generator_Elec::update_failsafes()
{
    AP_BattMonitor::Failsafe failsafe = AP_BattMonitor::Failsafe::None;

    AP_Generator *generator = AP::generator();

    // Only check for failsafes on the electrical moniter
    // no point in having the same failsafe on two battery monitors
    if (generator != nullptr) {
        failsafe = generator->update_failsafes();
    }
    return MAX(AP_BattMonitor_Backend::update_failsafes(), failsafe);
}
#endif  // AP_BATTERY_GENERATOR_ENABLED
