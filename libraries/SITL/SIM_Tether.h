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
   Simulate a tethered vehicle.
   Models the forces exerted by the tether and reports them to the vehicle simulation. The dynamics are not accurate but provide a very rough approximation intended to test a "stuck tether".
 */

#pragma once

#include "SIM_config.h"

#if AP_SIM_TETHER_ENABLED

#include <AP_HAL/utility/Socket_native.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/Location.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

namespace SITL {

// TetherSim simulates a tethered to a vehicle
class TetherSim {
public:
    // constructor
    TetherSim();

    // Update the tether simulation state using the vehicle's earth-frame position
    void update(const Location& veh_pos);

    // Retrieve earth-frame forces on the vehicle due to the tether
    // Returns true on success and fills in the forces_ef argument; false on failure
    bool get_forces_on_vehicle(Vector3f& forces_ef) const;

    // Parameter table for configuration
    static const struct AP_Param::GroupInfo var_info[];

private:
    // Parameters
    AP_Int8 enable;                  // Enable or disable the tether simulation
    AP_Float tether_linear_density;  // Linear mass density of the tether in kg/m
    AP_Float max_line_length;        // Maximum allowed tether length in meters
    AP_Int8 sys_id;                  // MAVLink system ID for GCS reporting
    AP_Int8 tether_stuck;            // Set to 1 to simulate a stuck tether
    AP_Float tether_spring_constant; // Spring constant for modeling tether stretch
    AP_Float tether_damping_constant; // Damping constant

    // Send MAVLink messages to the GCS
    void send_report();

    // Write tether simulation state to onboard log
    void write_log();

    // Retrieve the location of the tether anchor point on the ground
    // Returns true on success and fills in the tether_anchor_loc argument; false on failure
    bool get_tether_ground_location(Location& tether_anchor_loc) const;

    // Update forces exerted by the tether based on the vehicle's position
    // Takes the vehicle's position and the simulation time step (dt) as inputs
    void update_tether_force(const Location& veh_pos, float dt);

    // Socket connection variables for MAVLink communication
    const char *target_address = "127.0.0.1"; // Address for MAVLink socket communication
    const uint16_t target_port = 5763;        // Port for MAVLink socket communication
    SocketAPM_native mav_socket { false };   // Socket for MAVLink communication
    bool initialised;                        // True if the simulation class is initialized
    uint32_t last_update_us;                 // Timestamp of the last update in microseconds

    // MAVLink reporting variables
    const float reporting_period_ms = 100;   // Reporting period in milliseconds
    uint32_t last_report_ms;                // Timestamp of the last MAVLink report sent to GCS
    uint32_t last_heartbeat_ms;             // Timestamp of the last MAVLink heartbeat sent to GCS
    bool mavlink_connected;                 // True if MAVLink connection is established
    mavlink_status_t mav_status;            // Status of MAVLink communication

    // Tether simulation variables
    float prev_stretch = 0.0f;                // store stretch beyond maximum in last calculation
    Vector3f veh_forces_ef;                   // Effective Earth-frame forces on the vehicle due to the tether
    Vector3f veh_forces_teth;                 // Earth-frame forces on the vehicle due to the spring and damping forces on tether
    float tether_length = 0.0f;               // Current tether length in meters
    float tether_not_stuck_length = 0.0f;     // Tether length when the tether is not stuck
};

}  // namespace SITL

#endif  // AP_SIM_TETHER_ENABLED
