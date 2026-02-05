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
  battery model for electric aircraft
*/

#include "SIM_Battery.h"
#include <float.h>
#include <AP_Math/AP_Math.h>

using namespace SITL;

/*
  state of charge table for a single cell battery.
 */
static const struct {
    float volt_per_cell;
    float soc_pct;
} soc_table[] = {
    { 4.173, 100 },
    { 4.112, 96.15 },
    { 4.085, 92.31 },
    { 4.071, 88.46 },
    { 4.039, 84.62 },
    { 3.987, 80.77 },
    { 3.943, 76.92 },
    { 3.908, 73.08 },
    { 3.887, 69.23 },
    { 3.854, 65.38 },
    { 3.833, 61.54 },
    { 3.801, 57.69 },
    { 3.783, 53.85 },
    { 3.742, 50 },
    { 3.715, 46.15 },
    { 3.679, 42.31 },
    { 3.636, 38.46 },
    { 3.588, 34.62 },
    { 3.543, 30.77 },
    { 3.503, 26.92 },
    { 3.462, 23.08 },
    { 3.379, 19.23 },
    { 3.296, 15.38 },
    { 3.218, 11.54 },
    { 3.165, 7.69 },
    { 3.091, 3.85 },
    { 2.977, 2.0 },
    { 2.8,   1.5 },
    { 2.7,   1.3 },
    { 2.5,   1.2 },
    { 2.3,   1.1 },
    { 2.1,   1.0 },
    { 1.9,   0.9 },
    { 1.6,   0.8 },
    { 1.3,   0.7 },
    { 1.0,   0.6 },
    { 0.6,   0.4 },
    { 0.3,   0.2 },
    { 0.01,  0.01},
    { 0.001, 0.001 }};

/*
  use table to get resting voltage from remaining capacity
 */
float Battery::get_resting_voltage(void) const
{
    if (capacity_is_unlimited()) {
        return max_voltage;
    }
    float charge_pct = 100 * remaining_Ah / capacity_Ah;
    const float max_cell_voltage = soc_table[0].volt_per_cell;
    const float min_cell_voltage = soc_table[ARRAY_SIZE(soc_table) - 1].volt_per_cell;
    for (uint8_t i=1; i<ARRAY_SIZE(soc_table); i++) {
        if (charge_pct >= soc_table[i].soc_pct) {
            // linear interpolation between table rows
            float dv1 = charge_pct - soc_table[i].soc_pct;
            float dv2 = soc_table[i-1].soc_pct - soc_table[i].soc_pct;
            float vpc1 = soc_table[i].volt_per_cell;
            float vpc2 = soc_table[i-1].volt_per_cell;
            float cell_volt = vpc1 + (dv1 / dv2) * (vpc2 - vpc1);
            return (cell_volt / max_cell_voltage) * max_voltage;
        }
    }
    // off the bottom of the table
    return min_cell_voltage;
}

/*
  return remaining Amp-hours (aka "charge", "state of charge") corresponding to a voltage

  this is const for readability: it has no "side effects"
 */
float Battery::compute_remaining_ah(float voltage) const
{
    if (capacity_is_unlimited()) {
        return FLT_MAX;
    }

    const float max_cell_voltage = soc_table[0].volt_per_cell;
    float cell_volt = (voltage / max_voltage) * max_cell_voltage;

    for (uint8_t i=1; i<ARRAY_SIZE(soc_table); i++) {
        if (cell_volt >= soc_table[i].volt_per_cell) {
            // linear interpolation between table rows
            float dv1 = cell_volt - soc_table[i].volt_per_cell;
            float dv2 = soc_table[i-1].volt_per_cell - soc_table[i].volt_per_cell;
            float soc1 = soc_table[i].soc_pct;
            float soc2 = soc_table[i-1].soc_pct;
            float soc = soc1 + (dv1 / dv2) * (soc2 - soc1);
            return capacity_Ah * soc * 0.01;
        }
    }
    // off the bottom of the table
    return 0.0f;
}

void Battery::set_remaining_ah(void)
{
    remaining_Ah = compute_remaining_ah(voltage_set);
}

// Reminder: capacity <= 0 means **unlimited**
void Battery::setup(float _capacity_Ah, float _resistance_ohm, float _max_voltage)
{
    capacity_Ah = _capacity_Ah;
    resistance_ohm = _resistance_ohm;
    max_voltage = _max_voltage;

    voltage_set = max_voltage;
    voltage_filter.reset(voltage_set);
    set_remaining_ah();
}

void Battery::maybe_reset(float desired_voltage, float desired_capacity_Ah)
{
    const bool reset_not_needed = (is_equal(voltage_set, desired_voltage)
                                   && is_equal(capacity_Ah, desired_capacity_Ah));
    if (reset_not_needed) {
        return;
    }

    capacity_Ah = desired_capacity_Ah;
    // a negative desired voltage is unexpected, but not problematic
    voltage_set = MIN(desired_voltage, max_voltage);
    voltage_filter.reset(voltage_set);
    set_remaining_ah();
}

void Battery::consume_energy(float current_amps)
{
    uint64_t now = AP_HAL::micros64();
    float dt = (now - last_us) * 1.0e-6;
    if (dt > 0.1) {
        // we stopped updating
        dt = 0;
    }
    last_us = now;
    float delta_Ah = current_amps * dt / 3600;
    remaining_Ah -= delta_Ah;
    remaining_Ah = MAX(0, remaining_Ah);

    float voltage_delta = current_amps * resistance_ohm;
    float sagged_voltage = get_resting_voltage() - voltage_delta;
    voltage_filter.apply(sagged_voltage, dt);

    update_temperature(current_amps, now);
}

void Battery::update_temperature(float current_amps, uint64_t now)
{
    const float dt = (now - temperature.last_update_micros) * 1.0e-6;
    temperature.last_update_micros = now;

    // temperature growth model

    // thermal_capacity value chosen to match previous steady-state behavior at 28amps
    // (reminder: thermal_capacity = mass * specific_heat)
    const float inverse_of_thermal_capacity = 0.36f;  // use inverse so we can multiply, not divide
    temperature.kelvin += (current_amps * current_amps) * resistance_ohm * dt * inverse_of_thermal_capacity;

    // temperature decay model: first-order
    temperature.kelvin -= (temperature.kelvin - (ambient_temperature + 273.15f)) * 0.10f * dt;
}
