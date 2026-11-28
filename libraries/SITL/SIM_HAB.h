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
  a generic high altitude balloon simulator, with buoyancy physics and
  altitude control by gas venting and ballast release
*/

#pragma once

#include "SIM_config.h"

#if AP_SIM_HAB_ENABLED

#include "SIM_Aircraft.h"
#include <AP_Param/AP_Param.h>

namespace SITL {

class HAB : public Aircraft {
public:
    HAB(const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return NEW_NOTHROW HAB(frame_str);
    }

    static const struct AP_Param::GroupInfo var_info[];

    enum class EnvelopeType : uint8_t {
        ZERO_PRESSURE = 0,  // open duct, excess gas spills once the envelope is full
        SEALED = 1,         // closed envelope, builds superpressure up to the relief pressure
    };

    // servo output indexes used by the model
    enum Output : uint8_t {
        OUTPUT_VENT = 0,       // gas vent valve, proportional
        OUTPUT_BALLAST = 1,    // ballast dispenser, on above mid-range
        OUTPUT_TERMINATE = 2,  // flight termination, releases the lift gas
        OUTPUT_RELEASE = 3,    // launch release
        OUTPUT_TERMINATE_BACKUP = 4,  // second flight termination device
        OUTPUT_GRIPPER = 5,           // payload gripper, released at the bottom of its range
        OUTPUT_PAYLOAD_RELEASE = 6,   // payload release
    };

private:
    static constexpr uint8_t num_wind_layers = 4;

    // convert a servo output to a 0..1 demand
    float output_demand(const struct sitl_input &input, Output idx) const;

    // 0..1 demand from a servo output or from a GPIO pin, as used by a relay
    float actuator_demand(const struct sitl_input &input, Output idx, int8_t pin) const;

    // molar mass of the lift gas in kg/mol, kept lighter than air
    float gas_molar_mass_kg() const;

    // set the lift gas fill for the configured free lift at the launch site
    void fill_at_launch(float pressure, float temperature_K);

    // update gas temperature, volume, superpressure and actuators
    void update_gas(const struct sitl_input &input, float pressure, float temperature_K, float dt);

    // gas temperature offset the heat exchange is heading for
    float equilibrium_superheat() const;

    // replace the horizontal wind with the layered profile
    void update_layered_wind(float alt_amsl);

    // velocity of the air in a wind layer, north and east
    Vector2f layer_wind(uint8_t layer) const;

    // log model state
    void write_log(float temperature_K);

    AP_Enum<EnvelopeType> envelope_type;
    AP_Float volume_max;
    AP_Float dry_mass;
    AP_Float payload_mass;
    AP_Float ballast_initial;
    AP_Float gas_molar_mass;
    AP_Float free_lift;
    AP_Float drag_coefficient;
    AP_Float vent_rate;
    AP_Float ballast_rate;
    AP_Float relief_pressure;
    AP_Float superheat;
    AP_Float superheat_tau;
    AP_Float superheat_night;
    AP_Float day_length;
    AP_Float day_phase;
    AP_Int8  termination_fail;
    AP_Int8  vent_pin;
    AP_Int8  ballast_pin;
    AP_Int8  termination_pin;
    AP_Int8  termination_backup_pin;
    AP_Int8  actuator_fail;
    AP_Float payload_drop_mass;
    AP_Int8  payload_release_pin;
    AP_Float chute_cda;
    AP_Int8  wind_layers_enable;
    AP_Float wind_layer_alt[num_wind_layers];
    AP_Float wind_layer_speed[num_wind_layers];
    AP_Float wind_layer_dir[num_wind_layers];

    bool released;
    bool terminated;
    bool payload_dropped;

    // these change by very little each step, so are held as doubles
    double gas_mol;                 // lift gas in the envelope
    double ballast_mass;            // ballast remaining, kg
    double gas_temperature_K;       // lift gas temperature
    double time_since_release_s;

    float gas_volume;               // current gas volume, m^3
    float superpressure;            // envelope pressure above ambient, Pa
    uint32_t last_log_ms;
};

}

#endif // AP_SIM_HAB_ENABLED
