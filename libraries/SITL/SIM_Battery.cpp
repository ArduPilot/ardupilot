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
float Battery::get_resting_voltage(float charge_pct) const
{
    const float max_cell_voltage = soc_table[0].volt_per_cell;
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
    // off the bottom of the table, return a small non-zero to prevent math errors
    return 0.001;
}

/*
  use table to set initial state of charge from voltage
 */
void Battery::set_initial_SoC(float voltage)
{
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
            remaining_Ah = capacity_Ah * soc * 0.01;
            return;
        }
    }

    // off the bottom of the table
    remaining_Ah = 0;
}

void Battery::setup(float _capacity_Ah, float _resistance, float _max_voltage)
{
    capacity_Ah = _capacity_Ah;
    resistance = _resistance;
    max_voltage = _max_voltage;
}

void Battery::init_voltage(float voltage)
{
    voltage_filter.reset(voltage);
    voltage_set = voltage;
    set_initial_SoC(voltage);
}

void Battery::init_capacity(float capacity)
{
    capacity_Ah = capacity;
    set_initial_SoC(voltage_set);
}

void Battery::set_current(float current)
{
    uint64_t now = AP_HAL::micros64();
    float dt = (now - last_us) * 1.0e-6;
    if (dt > 0.1) {
        // we stopped updating
        dt = 0;
    }
    last_us = now;
    float delta_Ah = current * dt / 3600;
    remaining_Ah -= delta_Ah;
    remaining_Ah = MAX(0, remaining_Ah);

    float voltage_delta = current * resistance;
    float voltage;
    if (!is_positive(capacity_Ah)) {
        voltage = voltage_set;
    } else {
        voltage = get_resting_voltage(100 * remaining_Ah / capacity_Ah) - voltage_delta;
    }

    voltage_filter.apply(voltage, dt);

    {
        const uint64_t temperature_dt = now - temperature.last_update_micros;
        temperature.last_update_micros = now;
        // 1 amp*1 second == 0.1 degrees of energy.  Did those units hurt?
        temperature.kelvin += 0.1 * current * temperature_dt * 0.000001;
        // decay temperature at some %second towards ambient
        temperature.kelvin -= (temperature.kelvin - 273) * 0.10 * temperature_dt * 0.000001;
    }
}

float Battery::get_voltage(void) const
{
    return voltage_filter.get();
}
