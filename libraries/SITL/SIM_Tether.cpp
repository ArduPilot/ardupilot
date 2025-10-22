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
  simulate a static tether attached to the vehicle and ground
*/

#include "SIM_config.h"

#if AP_SIM_TETHER_ENABLED

#include "SIM_Tether.h"
#include "SITL.h"
#include <stdio.h>
#include "SIM_Aircraft.h"
#include <AP_HAL_SITL/SITL_State.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>

using namespace SITL;

// TetherSim parameters
const AP_Param::GroupInfo TetherSim::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Tether Simulation Enable/Disable
    // @Description: Enable or disable the tether simulation
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE",  1, TetherSim,  enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: DENSITY
    // @DisplayName: Tether Wire Density
    // @Description: Linear mass density of the tether wire
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("DENSITY",  2, TetherSim,  tether_linear_density, 0.0167),

    // @Param: LINELEN
    // @DisplayName: Tether Maximum Line Length
    // @Description: Maximum length of the tether line in meters
    // @Units: m
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("LINELEN", 3, TetherSim,  max_line_length, 100.0),

    // @Param: SYSID
    // @DisplayName: Tether Simulation MAVLink System ID
    // @Description: MAVLink system ID for the tether simulation, used to distinguish it from other systems on the network
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("SYSID",   4, TetherSim,  sys_id, 2),

    // @Param: STUCK
    // @DisplayName: Tether Stuck Enable/Disable
    // @Description: Enable or disable a stuck tether simulation
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("STUCK",   5, TetherSim,  tether_stuck, 0),

    // @Param: SPGCNST
    // @DisplayName: Tether Spring Constant
    // @Description: Spring constant for the tether to simulate elastic forces when stretched beyond its maximum length
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("SPGCNST",   6, TetherSim,  tether_spring_constant, 100),

    // @Param: DMPCNST
    // @DisplayName: Tether Damping Constant
    // @Description: Damping constant for the tether to simulate resistance based on change in stretch
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("DMPCNST",   7, TetherSim,  tether_damping_constant, 10),

    AP_GROUPEND
};

// TetherSim handles interaction with main vehicle
TetherSim::TetherSim()
{
    AP_Param::setup_object_defaults(this, var_info);
}


void TetherSim::update(const Location& veh_pos)
{
    if (!enable) {
        return;
    }

    if (veh_pos.is_zero()) {
        return;
    }

    // initialise fixed tether ground location
    const uint32_t now_us = AP_HAL::micros();
    if (!initialised) {
        // capture EKF origin
        auto *sitl = AP::sitl();
        const Location ekf_origin = sitl->state.home;
        if (ekf_origin.lat == 0 && ekf_origin.lng == 0) {
            return;
        }

        // more initialisation
        last_update_us = now_us;
        initialised = true;
    }

    // calculate dt and update tether forces
    const float dt = (now_us - last_update_us)*1.0e-6;
    last_update_us = now_us;

    update_tether_force(veh_pos, dt);

    // send tether location to GCS at 5hz
    // TO-Do: add provision to make the tether movable
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_report_ms >= reporting_period_ms) {
        last_report_ms = now_ms;
        send_report();
        write_log();
    }
}

// get earth-frame forces on the vehicle from the tether
// returns true on success and fills in forces_ef argument, false on failure
bool TetherSim::get_forces_on_vehicle(Vector3f& forces_ef) const
{
    if (!enable) {
        return false;
    }

    forces_ef = veh_forces_ef;
    return true;
}

// send a report to the vehicle control code over MAVLink
void TetherSim::send_report(void)
{
    if (!mavlink_connected && mav_socket.connect(target_address, target_port)) {
        ::printf("Tether System connected to %s:%u\n", target_address, (unsigned)target_port);
        mavlink_connected = true;
    }
    if (!mavlink_connected) {
        return;
    }

    // get current time
    uint32_t now_ms = AP_HAL::millis();

    // send heartbeat at 1hz
    const uint8_t component_id = MAV_COMP_ID_USER11;
    if (now_ms - last_heartbeat_ms >= 1000) {
        last_heartbeat_ms = now_ms;

        const mavlink_heartbeat_t heartbeat{
            custom_mode: 0,
            type : MAV_TYPE_GROUND_ROVER,
            autopilot : MAV_AUTOPILOT_INVALID,
            base_mode: 0,
            system_status: 0,
            mavlink_version: 0,
        };

        mavlink_message_t msg;
        mavlink_msg_heartbeat_encode_status(
            sys_id.get(),
            component_id,
            &mav_status,
            &msg,
            &heartbeat);
        uint8_t buf[300];
        const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        mav_socket.send(buf, len);
    }

    // send a GLOBAL_POSITION_INT messages
    {
        Location tether_anchor_loc;
        int32_t alt_amsl_cm, alt_rel_cm;
        if (!get_tether_ground_location(tether_anchor_loc) ||
            !tether_anchor_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, alt_amsl_cm) ||
            !tether_anchor_loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, alt_rel_cm)) {
            return;
        }
        const mavlink_global_position_int_t global_position_int{
            time_boot_ms: now_ms,
            lat: tether_anchor_loc.lat,
            lon: tether_anchor_loc.lng,
            alt: alt_amsl_cm * 10,              // amsl alt in mm
            relative_alt: alt_rel_cm * 10,      // relative alt in mm
            vx: 0,  // velocity in cm/s
            vy: 0,  // velocity in cm/s
            vz: 0,  // velocity in cm/s
            hdg: 0                              // heading in centi-degrees
        };
        mavlink_message_t msg;
        mavlink_msg_global_position_int_encode_status(sys_id, component_id, &mav_status, &msg, &global_position_int);
        uint8_t buf[300];
        const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        if (len > 0) {
            mav_socket.send(buf, len);
        }
    }
}

void TetherSim::write_log()
{
#if HAL_LOGGING_ENABLED
    // write log of tether state
    // @LoggerMessage: TETH
    // @Description: Tether state
    // @Field: TimeUS: Time since system startup
    // @Field: Len: Tether length
    // @Field: VFN: Force on vehicle in North direction
    // @Field: VFE: Force on vehicle in East direction
    // @Field: VFD: Force on vehicle in Down direction
    AP::logger().WriteStreaming("TETH",
                                "TimeUS,Len,VFN,VFE,VFD",  // labels
                                "s----",  // units
                                "F----",    // multipliers
                                "Qffff",     // format
                                AP_HAL::micros64(),
                                (float)tether_length,
                                (double)veh_forces_ef.x,
                                (double)veh_forces_ef.y,
                                (double)veh_forces_ef.z);
#endif
}
// returns true on success and fills in tether_ground_loc argument, false on failure
bool TetherSim::get_tether_ground_location(Location& tether_ground_loc) const
{
    // get EKF origin
    auto *sitl = AP::sitl();
    if (sitl == nullptr) {
        return false;
    }
    const Location ekf_origin = sitl->state.home;
    if (ekf_origin.lat == 0 && ekf_origin.lng == 0) {
        return false;
    }

    tether_ground_loc = ekf_origin;
    return true;
}

void TetherSim::update_tether_force(const Location& veh_pos, float dt)
{

    Location tether_anchor_loc;
    if (!get_tether_ground_location(tether_anchor_loc)) {
        return;
    }

    // Step 1: Calculate the vector from the tether anchor to the vehicle
    Vector3f tether_vector = veh_pos.get_distance_NED(tether_anchor_loc);
    tether_length = tether_vector.length();

    // Step 2: Check if tether is taut (length exceeds maximum allowed length) or stuck
    if (tether_length > max_line_length) {

        // Calculate the stretch beyond the maximum length
        float stretch = MAX(tether_length - max_line_length, 0.0f);

        // Calculate the stretch rate beyond the maximum length
        float stretch_rate = (stretch - prev_stretch)/dt;

        // Apply a spring and damping penalty force proportional to the stretch
        float penalty_force_magnitude = tether_spring_constant * stretch + tether_damping_constant * stretch_rate;

        // Direction of force is along the tether, pulling toward the anchor
        veh_forces_teth = tether_vector.normalized() * penalty_force_magnitude;

        prev_stretch = stretch;

    } else if (tether_stuck) {

        // Calculate the stretch beyond the maximum length
        float stretch = MAX(tether_length - tether_not_stuck_length, 0.0f);

        // Calculate the stretch rate beyond the maximum length
        float stretch_rate = (stretch - prev_stretch)/dt;

        // Apply a spring-like penalty force proportional to the stretch
        float penalty_force_magnitude = tether_spring_constant * stretch + tether_damping_constant * stretch_rate;

        // Direction of force is directly along the tether, towards the tether anchor point
        veh_forces_teth = tether_vector.normalized() * penalty_force_magnitude;

        prev_stretch = stretch;

    } else {
        tether_not_stuck_length = tether_length;
        veh_forces_teth.zero();
    }

    // Step 3: Calculate the weight of the tether being lifted
    // The weight is proportional to the current tether length
    const float tether_weight_force = tether_linear_density * tether_length * GRAVITY_MSS;

    // Step 4: Calculate the tension force
    Vector3f tension_force_NED = tether_vector.normalized() * tether_weight_force;

    // Step 5: Apply the total force to the vehicle
    veh_forces_ef = veh_forces_teth + tension_force_NED;
}

#endif
