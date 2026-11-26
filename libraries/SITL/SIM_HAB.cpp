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
  a generic high altitude balloon simulator

  The balloon is described by physical parameters so that one model
  covers a range of envelope types and sizes. Vertical motion comes
  from the balance of buoyancy, weight and drag. Altitude is
  controlled by venting lift gas (descend) and releasing ballast
  (climb), both of which are consumed.

  Simplifications:
   - the gas cools and warms adiabatically as the balloon climbs and
     descends, and relaxes towards ambient plus a superheat offset
     with a first order lag. There is no radiative model.
   - the envelope is treated as a sphere of the current gas volume for
     drag
   - gas leakage is not modelled
*/

#include "SIM_HAB.h"

#if AP_SIM_HAB_ENABLED

#include <AP_Baro/AP_Baro.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>


using namespace SITL;

extern const AP_HAL::HAL& hal;

// molar mass of dry air, kg/mol
static constexpr float AIR_MOLAR_MASS = 0.0289644f;
// universal gas constant, J/(mol.K)
static constexpr float GAS_CONSTANT = 8.3144621f;
// added mass coefficient for a sphere
static constexpr float ADDED_MASS_COEFF = 0.5f;
// molar heat capacity at constant pressure, J/(mol.K)
static constexpr float CP_MONATOMIC = 20.8f;
static constexpr float CP_DIATOMIC = 29.1f;

// SITL HAB parameters
const AP_Param::GroupInfo HAB::var_info[] = {
    // @Param: ENV_TYPE
    // @DisplayName: envelope type
    // @Description: A zero pressure envelope spills excess gas once it is full. A sealed envelope keeps its gas and builds superpressure up to the relief pressure.
    // @Values: 0:Zero pressure,1:Sealed
    AP_GROUPINFO("ENV_TYPE", 1, HAB, envelope_type, float(EnvelopeType::ZERO_PRESSURE)),

    // @Param: VOL_MAX
    // @DisplayName: maximum envelope volume
    // @Description: volume of the envelope when fully inflated, in cubic metres
    // @Range: 1 20000
    AP_GROUPINFO("VOL_MAX", 2, HAB, volume_max, 60),

    // @Param: MASS
    // @DisplayName: dry mass
    // @Description: mass of the envelope, rigging and avionics, not including payload, ballast or lift gas
    // @Units: kg
    // @Range: 0.1 500
    AP_GROUPINFO("MASS", 3, HAB, dry_mass, 3.5),

    // @Param: PAYLOAD
    // @DisplayName: payload mass
    // @Description: mass of the payload
    // @Units: kg
    AP_GROUPINFO("PAYLOAD", 4, HAB, payload_mass, 2.0),

    // @Param: BALLAST
    // @DisplayName: ballast mass
    // @Description: mass of droppable ballast loaded at launch
    // @Units: kg
    AP_GROUPINFO("BALLAST", 5, HAB, ballast_initial, 1.5),

    // @Param: GAS_MW
    // @DisplayName: lift gas molar mass
    // @Description: molar mass of the lift gas in grams per mole. Helium is 4.003, hydrogen is 2.016.
    // @Range: 2 20
    AP_GROUPINFO("GAS_MW", 6, HAB, gas_molar_mass, 4.003),

    // @Param: FREE_LFT
    // @DisplayName: free lift
    // @Description: lift in excess of the total launch mass, as a proportion of that mass. This sets the amount of lift gas loaded at launch.
    // @Range: 0.05 1
    AP_GROUPINFO("FREE_LFT", 7, HAB, free_lift, 0.2),

    // @Param: CD
    // @DisplayName: drag coefficient
    // @Description: drag coefficient of the envelope, referenced to the frontal area of a sphere of the current gas volume
    // @Range: 0 2
    AP_GROUPINFO("CD", 8, HAB, drag_coefficient, 0.5),

    // @Param: VENT_RT
    // @DisplayName: vent valve flow rate
    // @Description: volume flow of lift gas through the fully open vent valve in cubic metres per second, measured at envelope conditions
    // @Range: 0 1
    AP_GROUPINFO("VENT_RT", 9, HAB, vent_rate, 0.01),

    // @Param: BAL_RT
    // @DisplayName: ballast release rate
    // @Description: mass flow of ballast while the dispenser is running, in kilograms per second
    // @Range: 0 1
    AP_GROUPINFO("BAL_RT", 10, HAB, ballast_rate, 0.005),

    // @Param: DP_MAX
    // @DisplayName: relief pressure
    // @Description: superpressure at which a sealed envelope relieves gas. Not used by a zero pressure envelope.
    // @Units: Pa
    AP_GROUPINFO("DP_MAX", 11, HAB, relief_pressure, 400),

    // @Param: SUPERHT
    // @DisplayName: gas superheat
    // @Description: equilibrium temperature of the lift gas relative to ambient, in kelvin. Positive for solar heating. This is the daytime value when the day and night cycle is enabled.
    AP_GROUPINFO("SUPERHT", 12, HAB, superheat, 0),

    // @Param: HEAT_TAU
    // @DisplayName: gas thermal time constant
    // @Description: time constant with which the gas temperature approaches its equilibrium. A longer time constant gives more cooling of the gas during ascent.
    // @Units: s
    // @Range: 1 3600
    AP_GROUPINFO("HEAT_TAU", 13, HAB, superheat_tau, 300),

    // @Param: CHUTE
    // @DisplayName: parachute drag area
    // @Description: drag coefficient multiplied by area of the recovery parachute, used after flight termination
    // @Units: m.m
    // @Range: 0 50
    AP_GROUPINFO("CHUTE", 14, HAB, chute_cda, 2.0),

    // @Param: WND_ENA
    // @DisplayName: layered wind enable
    // @Description: replace the horizontal SITL wind with a profile interpolated between the wind layers. Layers must be in order of increasing altitude. The wind below the first layer and above the last layer is held constant.
    // @Values: 0:Disabled,1:Enabled
    AP_GROUPINFO("WND_ENA", 15, HAB, wind_layers_enable, 0),

    // @Param: W1_ALT
    // @DisplayName: wind layer 1 altitude
    // @Description: altitude above mean sea level of wind layer 1
    // @Units: m
    AP_GROUPINFO("W1_ALT", 16, HAB, wind_layer_alt[0], 0),

    // @Param: W1_SPD
    // @DisplayName: wind layer 1 speed
    // @Description: wind speed in wind layer 1
    // @Units: m/s
    AP_GROUPINFO("W1_SPD", 17, HAB, wind_layer_speed[0], 0),

    // @Param: W1_DIR
    // @DisplayName: wind layer 1 direction
    // @Description: direction the wind is coming from in wind layer 1
    // @Units: deg
    AP_GROUPINFO("W1_DIR", 18, HAB, wind_layer_dir[0], 0),

    // @Param: W2_ALT
    // @DisplayName: wind layer 2 altitude
    // @Description: altitude above mean sea level of wind layer 2
    // @Units: m
    AP_GROUPINFO("W2_ALT", 19, HAB, wind_layer_alt[1], 0),

    // @Param: W2_SPD
    // @DisplayName: wind layer 2 speed
    // @Description: wind speed in wind layer 2
    // @Units: m/s
    AP_GROUPINFO("W2_SPD", 20, HAB, wind_layer_speed[1], 0),

    // @Param: W2_DIR
    // @DisplayName: wind layer 2 direction
    // @Description: direction the wind is coming from in wind layer 2
    // @Units: deg
    AP_GROUPINFO("W2_DIR", 21, HAB, wind_layer_dir[1], 0),

    // @Param: W3_ALT
    // @DisplayName: wind layer 3 altitude
    // @Description: altitude above mean sea level of wind layer 3
    // @Units: m
    AP_GROUPINFO("W3_ALT", 22, HAB, wind_layer_alt[2], 0),

    // @Param: W3_SPD
    // @DisplayName: wind layer 3 speed
    // @Description: wind speed in wind layer 3
    // @Units: m/s
    AP_GROUPINFO("W3_SPD", 23, HAB, wind_layer_speed[2], 0),

    // @Param: W3_DIR
    // @DisplayName: wind layer 3 direction
    // @Description: direction the wind is coming from in wind layer 3
    // @Units: deg
    AP_GROUPINFO("W3_DIR", 24, HAB, wind_layer_dir[2], 0),

    // @Param: W4_ALT
    // @DisplayName: wind layer 4 altitude
    // @Description: altitude above mean sea level of wind layer 4
    // @Units: m
    AP_GROUPINFO("W4_ALT", 25, HAB, wind_layer_alt[3], 0),

    // @Param: W4_SPD
    // @DisplayName: wind layer 4 speed
    // @Description: wind speed in wind layer 4
    // @Units: m/s
    AP_GROUPINFO("W4_SPD", 26, HAB, wind_layer_speed[3], 0),

    // @Param: W4_DIR
    // @DisplayName: wind layer 4 direction
    // @Description: direction the wind is coming from in wind layer 4
    // @Units: deg
    AP_GROUPINFO("W4_DIR", 27, HAB, wind_layer_dir[3], 0),

    // @Param: NIGHT_DT
    // @DisplayName: gas temperature offset at night
    // @Description: equilibrium temperature of the lift gas relative to ambient at night, in kelvin. Negative for radiative cooling. Used when the day and night cycle is enabled.
    AP_GROUPINFO("NIGHT_DT", 28, HAB, superheat_night, 0),

    // @Param: DAY_LEN
    // @DisplayName: length of the day and night cycle
    // @Description: length of one cycle of day and night. The gas temperature offset moves between the daytime and night values over the cycle, with half the cycle in daylight. Zero disables the cycle and the daytime value is used throughout.
    // @Units: s
    AP_GROUPINFO("DAY_LEN", 29, HAB, day_length, 0),

    // @Param: DAY_PHS
    // @DisplayName: phase of the day at release
    // @Description: point in the day and night cycle at which the balloon is released, as a proportion of the cycle. Zero is sunrise, 0.25 is the middle of the day and 0.5 is sunset.
    // @Range: 0 1
    AP_GROUPINFO("DAY_PHS", 30, HAB, day_phase, 0.25),

    // @Param: TRM_FAIL
    // @DisplayName: flight termination failure
    // @Description: flight termination devices which do nothing when operated
    // @Bitmask: 0:Termination,1:Termination backup
    AP_GROUPINFO("TRM_FAIL", 31, HAB, termination_fail, 0),

    // @Param: VNT_PIN
    // @DisplayName: vent valve pin
    // @Description: GPIO pin which opens the vent valve fully, as an alternative to the servo output. Set to the pin of a relay. The device is operated while the pin is high, so the relay must not be inverted. -1 to disable.
    // @Range: -1 15
    AP_GROUPINFO("VNT_PIN", 32, HAB, vent_pin, -1),

    // @Param: BAL_PIN
    // @DisplayName: ballast release pin
    // @Description: GPIO pin which runs the ballast dispenser, as an alternative to the servo output. -1 to disable.
    // @Range: -1 15
    AP_GROUPINFO("BAL_PIN", 33, HAB, ballast_pin, -1),

    // @Param: TRM_PIN
    // @DisplayName: termination pin
    // @Description: GPIO pin which operates the flight termination device, as an alternative to the servo output. -1 to disable.
    // @Range: -1 15
    AP_GROUPINFO("TRM_PIN", 34, HAB, termination_pin, -1),

    // @Param: TR2_PIN
    // @DisplayName: termination backup pin
    // @Description: GPIO pin which operates the second flight termination device, as an alternative to the servo output. -1 to disable.
    // @Range: -1 15
    AP_GROUPINFO("TR2_PIN", 35, HAB, termination_backup_pin, -1),

    // @Param: ACT_FAIL
    // @DisplayName: actuator failure
    // @Description: actuators which do nothing when operated
    // @Bitmask: 0:Vent valve,1:Ballast release
    AP_GROUPINFO("ACT_FAIL", 36, HAB, actuator_fail, 0),

    // @Param: DROP
    // @DisplayName: payload dropped
    // @Description: mass of the payload which leaves the balloon when the payload release is operated
    // @Units: kg
    // @Range: 0 100
    AP_GROUPINFO("DROP", 37, HAB, payload_drop_mass, 0),

    // @Param: REL_PIN
    // @DisplayName: payload release pin
    // @Description: GPIO pin which operates the payload release, as an alternative to the servo output. -1 to disable.
    // @Range: -1 15
    AP_GROUPINFO("REL_PIN", 38, HAB, payload_release_pin, -1),

    AP_GROUPEND
};

HAB::HAB(const char *frame_str) :
    Aircraft(frame_str)
{
    AP::sitl()->models.hab_ptr = this;
    AP_Param::setup_object_defaults(this, var_info);
    ground_behavior = GROUND_BEHAVIOR_NO_MOVEMENT;
}

// convert a servo output to a 0..1 demand
float HAB::output_demand(const struct sitl_input &input, Output idx) const
{
    return constrain_float((input.servos[idx] - 1000) * 0.001f, 0, 1);
}

// molar mass of the lift gas in kg/mol, kept lighter than air
float HAB::gas_molar_mass_kg() const
{
    return constrain_float(gas_molar_mass * 0.001f, 0.001f, AIR_MOLAR_MASS * 0.9f);
}

// 0..1 demand from a servo output or from a GPIO pin, as used by a relay
float HAB::actuator_demand(const struct sitl_input &input, Output idx, int8_t pin) const
{
    if (pin >= 0 && pin < 16 && (AP::sitl()->pin_mask.get() & (1U << pin)) != 0) {
        return 1;
    }
    return output_demand(input, idx);
}

// set the lift gas fill for the configured free lift at the launch site
void HAB::fill_at_launch(float pressure, float temperature_K)
{
    ballast_mass = MAX(ballast_initial.get(), 0.0f);
    gas_temperature_K = temperature_K;
    time_since_release_s = 0;

    // with gas and air at the same temperature and pressure each mole
    // of lift gas displaces a mole of air, so the net lift per mole
    // does not depend on altitude
    const float launch_mass = dry_mass + payload_mass + ballast_mass;
    gas_mol = launch_mass * (1 + MAX(free_lift.get(), 0.0f)) / (AIR_MOLAR_MASS - gas_molar_mass_kg());

    // a fill that does not fit in the envelope at launch is spilled
    const double mol_max = pressure * MAX(volume_max.get(), 0.0f) / (GAS_CONSTANT * temperature_K);
    if (gas_mol > mol_max) {
        gas_mol = mol_max;
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SIM HAB envelope too small for the free lift");
    }
}

// update gas temperature, volume, superpressure and actuators
void HAB::update_gas(const struct sitl_input &input, float pressure, float temperature_K, float dt)
{
    // the gas does work on the surrounding air as it expands, so it
    // cools on the way up and warms on the way down. The pressure
    // change is hydrostatic in air, giving a lapse rate of M_air*g/cp.
    // A full sealed envelope holds the gas at constant volume, so no
    // work is done.
    if (superpressure <= 0) {
        const float cp = gas_molar_mass < 3 ? CP_DIATOMIC : CP_MONATOMIC;
        const float climb_rate = -velocity_ef.z;
        gas_temperature_K -= double(AIR_MOLAR_MASS * GRAVITY_MSS / cp * climb_rate) * dt;
    }

    // heat exchange takes the gas towards ambient plus the superheat
    time_since_release_s += dt;
    const float tau = MAX(superheat_tau.get(), 1.0f);
    gas_temperature_K += (double(temperature_K + equilibrium_superheat()) - gas_temperature_K) * MIN(dt / tau, 1.0f);
    gas_temperature_K = MAX(gas_temperature_K, 100.0);
    const float gas_T = gas_temperature_K;

    // ballast dispenser
    if (actuator_demand(input, OUTPUT_BALLAST, ballast_pin) > 0.5f && (actuator_fail & 2) == 0) {
        ballast_mass = MAX(ballast_mass - double(MAX(ballast_rate.get(), 0.0f) * dt), 0.0);
    }

    // vent valve, flow is volumetric at envelope conditions
    const float vent = (actuator_fail & 1) == 0 ? actuator_demand(input, OUTPUT_VENT, vent_pin) : 0;
    if (vent > 0) {
        const float mol_per_m3 = (pressure + superpressure) / (GAS_CONSTANT * gas_T);
        gas_mol = MAX(gas_mol - vent * MAX(vent_rate.get(), 0.0f) * mol_per_m3 * dt, 0.0);
    }

    // volume the gas would take up if unconstrained
    const float envelope_volume = MAX(volume_max.get(), 0.0f);
    const float volume_free = gas_mol * GAS_CONSTANT * gas_T / pressure;
    if (volume_free <= envelope_volume) {
        gas_volume = volume_free;
        superpressure = 0;
        return;
    }

    gas_volume = envelope_volume;
    const double mol_full = pressure * envelope_volume / (GAS_CONSTANT * gas_T);

    switch (EnvelopeType(envelope_type.get())) {
    case EnvelopeType::SEALED: {
        superpressure = pressure * (gas_mol / mol_full - 1);
        const float relief = MAX(relief_pressure.get(), 0.0f);
        if (superpressure > relief) {
            // relief valve holds the envelope at the relief pressure
            superpressure = relief;
            gas_mol = mol_full * (1 + relief / pressure);
        }
        break;
    }
    case EnvelopeType::ZERO_PRESSURE:
    default:
        // excess gas spills from the duct
        superpressure = 0;
        gas_mol = mol_full;
        break;
    }
}

// gas temperature offset the heat exchange is heading for
float HAB::equilibrium_superheat() const
{
    if (day_length <= 0) {
        return superheat;
    }
    // half of the cycle is daylight, with about an hour of a 24 hour
    // cycle taken to change over at sunrise and sunset
    const double cycles = time_since_release_s / day_length + day_phase;
    const float sunlit = constrain_float(0.5f + 4 * sinf(2 * M_PI * float(cycles - floor(cycles))), 0, 1);
    return superheat_night + (superheat - superheat_night) * sunlit;
}

// replace the horizontal wind with the layered profile
void HAB::update_layered_wind(float alt_amsl)
{
    // layers are in use up to the first that is not above the one before
    uint8_t num_layers = 1;
    while (num_layers < num_wind_layers &&
           wind_layer_alt[num_layers] > wind_layer_alt[num_layers-1]) {
        num_layers++;
    }

    // find the layers either side of the current altitude
    uint8_t upper = 0;
    while (upper < num_layers && alt_amsl >= wind_layer_alt[upper]) {
        upper++;
    }

    Vector2f wind;
    if (upper == 0) {
        wind = layer_wind(0);
    } else if (upper == num_layers) {
        // above the top layer
        wind = layer_wind(upper-1);
    } else {
        // interpolate the wind vector, so opposed layers pass through calm
        const uint8_t lower = upper - 1;
        const float p = (alt_amsl - wind_layer_alt[lower]) / (wind_layer_alt[upper] - wind_layer_alt[lower]);
        wind = layer_wind(lower) * (1 - p) + layer_wind(upper) * p;
    }

    wind_ef.x = wind.x;
    wind_ef.y = wind.y;
}

// velocity of the air in a wind layer, north and east
Vector2f HAB::layer_wind(uint8_t layer) const
{
    // direction is where the wind comes from
    const float dir_rad = radians(wind_layer_dir[layer]);
    return Vector2f(-wind_layer_speed[layer] * cosf(dir_rad),
                    -wind_layer_speed[layer] * sinf(dir_rad));
}

// log model state
void HAB::write_log(float temperature_K)
{
#if HAL_LOGGING_ENABLED
    const uint32_t now_ms = AP_HAL::millis();
    if (!released || now_ms - last_log_ms < 1000) {
        return;
    }
    last_log_ms = now_ms;

// @LoggerMessage: SHAB
// @Description: Simulated high altitude balloon state
// @Field: TimeUS: Time since system startup
// @Field: Vol: gas volume
// @Field: Mol: lift gas remaining
// @Field: Bal: ballast remaining
// @Field: dT: gas temperature relative to ambient
// @Field: dP: superpressure
// @Field: WN: wind velocity north
// @Field: WE: wind velocity east
    AP::logger().WriteStreaming("SHAB", "TimeUS,Vol,Mol,Bal,dT,dP,WN,WE",
                                "Qfffffff",
                                AP_HAL::micros64(),
                                gas_volume,
                                float(gas_mol),
                                float(ballast_mass),
                                gas_temperature_K - temperature_K,
                                superpressure,
                                wind_ef.x,
                                wind_ef.y);
#endif
}

/*
  update the balloon simulation by one time step
 */
void HAB::update(const struct sitl_input &input)
{
    const float dt = frame_time_us * 1.0e-6f;
    const float alt_amsl = location.alt * 0.01f;

    update_wind(input);
    if (wind_layers_enable > 0) {
        update_layered_wind(alt_amsl);
    }

    float pressure, temperature_K;
    AP_Baro::get_pressure_temperature_for_alt_amsl(alt_amsl, pressure, temperature_K);
    const float rho = AP_Baro::get_air_density_for_alt_amsl(alt_amsl);

    if (!released && hal.scheduler->is_system_initialized() &&
        output_demand(input, OUTPUT_RELEASE) > 0.8f) {
        fill_at_launch(pressure, temperature_K);
        released = true;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SIM HAB released, %.0f mol of lift gas", gas_mol);
    }

    const bool terminate =
        (actuator_demand(input, OUTPUT_TERMINATE, termination_pin) > 0.8f && (termination_fail & 1) == 0) ||
        (actuator_demand(input, OUTPUT_TERMINATE_BACKUP, termination_backup_pin) > 0.8f && (termination_fail & 2) == 0);
    if (released && !terminated && terminate) {
        terminated = true;
        gas_mol = 0;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SIM HAB terminated at %.0f m", alt_amsl);
    }

    // a gripper releases at the bottom of its range, and sits at the
    // middle until it is first operated
    const uint16_t gripper_pwm = input.servos[OUTPUT_GRIPPER];
    const bool let_go = (gripper_pwm > 800 && gripper_pwm < 1300) ||
        actuator_demand(input, OUTPUT_PAYLOAD_RELEASE, payload_release_pin) > 0.8f;
    if (released && !payload_dropped && let_go) {
        payload_dropped = true;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SIM HAB payload released at %.0f m", alt_amsl);
    }

    Vector3f rot_accel;

    if (!released) {
        // held at the launch site
        velocity_ef.zero();
        gyro.zero();
        accel_body = dcm.transposed() * Vector3f(0, 0, -GRAVITY_MSS);
    } else {
        float drag_area;
        if (terminated) {
            gas_volume = 0;
            superpressure = 0;
            drag_area = MAX(chute_cda.get(), 0.0f);
        } else {
            update_gas(input, pressure, temperature_K, dt);
            const float radius_m = cbrtf(gas_volume * 3 / (4 * M_PI));
            drag_area = MAX(drag_coefficient.get(), 0.0f) * M_PI * sq(radius_m);
        }

        const float gas_mass = gas_mol * gas_molar_mass_kg();
        const float payload = payload_dropped ? MAX(payload_mass - MAX(payload_drop_mass.get(), 0.0f), 0.0f) : payload_mass.get();
        mass = MAX(dry_mass + payload + ballast_mass + gas_mass, 0.1f);

        // the envelope has to accelerate the air around it
        const float displaced_mass = rho * gas_volume;
        const float inertial_mass = mass + ADDED_MASS_COEFF * displaced_mass;

        const Vector3f velocity_rel = velocity_ef - wind_ef;
        Vector3f force_ef = velocity_rel * (-0.5f * rho * drag_area * velocity_rel.length());
        // buoyancy and weight, earth frame is NED
        force_ef.z += (mass - displaced_mass) * GRAVITY_MSS;

        // update_dynamics() adds gravity to the acceleration we supply
        const Vector3f accel_ef = force_ef / inertial_mass - Vector3f(0, 0, GRAVITY_MSS);
        accel_body = dcm.transposed() * accel_ef;

        // the payload hangs below the envelope, so the system rights itself
        float roll, pitch, yaw;
        dcm.to_euler(&roll, &pitch, &yaw);
        rot_accel = Vector3f(-2 * roll, -2 * pitch, 0) - gyro * 2;
    }

    update_dynamics(rot_accel);

    // update lat/lon/altitude
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();

    update_battery();

    write_log(temperature_K);
}

#endif // AP_SIM_HAB_ENABLED
