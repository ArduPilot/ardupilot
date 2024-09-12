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
  simulate a slung payload
*/

#include "SIM_config.h"

#if AP_SIM_SLUNGPAYLOAD_ENABLED

#include "SIM_SlungPayload.h"
#include "SITL.h"
#include <stdio.h>
#include "SIM_Aircraft.h"
#include <AP_HAL_SITL/SITL_State.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>

using namespace SITL;

// SlungPayloadSim parameters
const AP_Param::GroupInfo SlungPayloadSim::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Slung Payload Sim enable/disable
    // @Description: Slung Payload Sim enable/disable
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE",  1, SlungPayloadSim,  enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: WEIGHT
    // @DisplayName: Slung Payload weight
    // @Description: Slung Payload weight in kg
    // @Units: kg
    // @Range: 0 15
    // @User: Advanced
    AP_GROUPINFO("WEIGHT",  2, SlungPayloadSim,  weight_kg, 1.0),

    // @Param: LINELEN
    // @DisplayName: Slung Payload line length
    // @Description: Slung Payload line length in meters
    // @Units: m
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("LINELEN", 3, SlungPayloadSim,  line_length, 30.0),

    // @Param: DRAG
    // @DisplayName: Slung Payload drag coefficient
    // @Description: Slung Payload drag coefficient.  Higher values increase drag and slow the payload more quickly
    // @Units: m
    // @Range: 0 10
    // @User: Advanced
    AP_GROUPINFO("DRAG",    4, SlungPayloadSim,  drag_coef, 1),

    // @Param: SYSID
    // @DisplayName: Slung Payload MAVLink system ID
    // @Description: Slung Payload MAVLink system id to distinguish it from others on the same network
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("SYSID",   5, SlungPayloadSim,  sys_id, 2),

    AP_GROUPEND
};

// SlungPayloadSim handles interaction with main vehicle
SlungPayloadSim::SlungPayloadSim()
{
    AP_Param::setup_object_defaults(this, var_info);
}

// update the SlungPayloadSim's state using the vehicle's earth-frame position, velocity, acceleration and wind
void SlungPayloadSim::update(const Vector3p& veh_pos, const Vector3f& veh_vel_ef, const Vector3f& veh_accel_ef, const Vector3f& wind_ef)
{
    if (!enable) {
        return;
    }

    // initialise slung payload location
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

    // calculate dt and update slung payload
    const float dt = (now_us - last_update_us)*1.0e-6;
    last_update_us = now_us;
    update_payload(veh_pos, veh_vel_ef, veh_accel_ef, wind_ef, dt);

    // send payload location to GCS at 5hz
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_report_ms >= reporting_period_ms) {
        last_report_ms = now_ms;
        send_report();
        write_log();
    }
}

// get earth-frame forces on the vehicle from slung payload
// returns true on success and fills in forces_ef argument, false on failure
bool SlungPayloadSim::get_forces_on_vehicle(Vector3f& forces_ef) const
{
    if (!enable) {
        return false;
    }

    forces_ef = veh_forces_ef;
    return true;
}

// send a report to the vehicle control code over MAVLink
void SlungPayloadSim::send_report(void)
{
    if (!mavlink_connected && mav_socket.connect(target_address, target_port)) {
        ::printf("SlungPayloadSim connected to %s:%u\n", target_address, (unsigned)target_port);
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
            type : MAV_TYPE_AIRSHIP,
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
        Location payload_loc;
        int32_t alt_amsl_cm, alt_rel_cm;
        if (!get_payload_location(payload_loc) ||
            !payload_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, alt_amsl_cm) ||
            !payload_loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, alt_rel_cm)) {
            return;
        }
        const mavlink_global_position_int_t global_position_int{
            time_boot_ms: now_ms,
            lat: payload_loc.lat,
            lon: payload_loc.lng,
            alt: alt_amsl_cm * 10,              // amsl alt in mm
            relative_alt: alt_rel_cm * 10,      // relative alt in mm
            vx: int16_t(velocity_NED.x * 100),  // velocity in cm/s
            vy: int16_t(velocity_NED.y * 100),  // velocity in cm/s
            vz: int16_t(velocity_NED.z * 100),  // velocity in cm/s
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

    // send ATTITUDE so MissionPlanner can display orientation
    {
        const mavlink_attitude_t attitude{
            time_boot_ms: now_ms,
            roll: 0,
            pitch: 0,
            yaw: 0,         // heading in radians
            rollspeed: 0,
            pitchspeed: 0,
            yawspeed: 0
        };
        mavlink_message_t msg;
        mavlink_msg_attitude_encode_status(
                sys_id,
                component_id,
                &mav_status,
                &msg,
                &attitude);
        uint8_t buf[300];
        const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        if (len > 0) {
            mav_socket.send(buf, len);
        }
    }
}

// write onboard log
void SlungPayloadSim::write_log()
{
#if HAL_LOGGING_ENABLED
    // write log of slung payload state
    // @LoggerMessage: SLUP
    // @Description: Slung payload
    // @Field: TimeUS: Time since system startup
    // @Field: Land: 1 if payload is landed, 0 otherwise
    // @Field: Tens: Tension ratio, 1 if line is taut, 0 if slack
    // @Field: Len: Line length
    // @Field: PN: Payload position as offset from vehicle in North direction
    // @Field: PE: Payload position as offset from vehicle in East direction
    // @Field: PD: Payload position as offset from vehicle in Down direction
    // @Field: VN: Payload velocity in North direction
    // @Field: VE: Payload velocity in East direction
    // @Field: VD: Payload velocity in Down direction
    // @Field: AN: Payload acceleration in North direction
    // @Field: AE: Payload acceleration in East direction
    // @Field: AD: Payload acceleration in Down direction
    // @Field: VFN: Force on vehicle in North direction
    // @Field: VFE: Force on vehicle in East direction
    // @Field: VFD: Force on vehicle in Down direction
    AP::logger().WriteStreaming("SLUP",
                                "TimeUS,Land,Tens,Len,PN,PE,PD,VN,VE,VD,AN,AE,AD,VFN,VFE,VFD",  // labels
                                "s-%mmmmnnnooo---",  // units
                                "F-20000000000000",  // multipliers
                                "Qbffffffffffffff",  // format
                                AP_HAL::micros64(),
                                (uint8_t)landed,
                                (float)tension_ratio,
                                (float)payload_to_veh.length(),
                                (double)-payload_to_veh.x,
                                (double)-payload_to_veh.y,
                                (double)-payload_to_veh.z,
                                (double)velocity_NED.x,
                                (double)velocity_NED.y,
                                (double)velocity_NED.z,
                                (double)accel_NED.x,
                                (double)accel_NED.y,
                                (double)accel_NED.z,
                                (double)veh_forces_ef.x,
                                (double)veh_forces_ef.y,
                                (double)veh_forces_ef.z);
#endif
}

// returns true on success and fills in payload_loc argument, false on failure
bool SlungPayloadSim::get_payload_location(Location& payload_loc) const
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

    // calculate location
    payload_loc = ekf_origin;
    payload_loc.offset(position_NED);
    return true;
}

// update the slung payloads position, velocity, acceleration
// vehicle position, velocity, acceleration and wind should be in earth-frame NED frame
void SlungPayloadSim::update_payload(const Vector3p& veh_pos, const Vector3f& veh_vel_ef, const Vector3f& veh_accel_ef,
                                     const Vector3f& wind_ef, float dt)
{
    // how we calculate the payload's position, velocity and acceleration
    //   1. update the payload's position, velocity using the previous iterations acceleration
    //   2. check that the payload does not fall below the terrain
    //   3. check if the line is taught and that the payload does not move more than the line length from the vehicle
    //   4. calculate gravity and drag forces on the payload
    //   5. calculate the tension force between the payload and vehicle including force countering gravity, drag and centripetal force
    //   6. update the payload's acceleration using the sum of the above forces

    // initialise position_NED from vehicle position
    if (position_NED.is_zero()) {
        if (!veh_pos.is_zero()) {
            position_NED = veh_pos;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SlungPayload: initialised at %f %f %f", position_NED.x, position_NED.y, position_NED.z);
        }
        return;
    }

    // integrate previous iterations acceleration into velocity and position
    velocity_NED += accel_NED * dt;
    position_NED += (velocity_NED * dt).todouble();

    // calculate distance from payload to vehicle
    payload_to_veh = veh_pos - position_NED;
    float payload_to_veh_length = payload_to_veh.length();

    // update landed state by checking if payload has dropped below terrain
    Location payload_loc;
    if (get_payload_location(payload_loc)) {
        int32_t alt_terrain_cm;
        bool landed_orig = landed;
        if (payload_loc.get_alt_cm(Location::AltFrame::ABOVE_TERRAIN, alt_terrain_cm)) {

            // landed if below terrain
            if (alt_terrain_cm <= 0) {
                landed = true;

                // raise payload to match terrain
                position_NED.z += (alt_terrain_cm * 0.01);

                // zero out velocity and acceleration in horizontal and downward direction
                velocity_NED.xy().zero();
                velocity_NED.z = MIN(velocity_NED.z, 0);
                accel_NED.xy().zero();
                accel_NED.z = MIN(accel_NED.z, 0);

                // zero out forces on vehicle
                veh_forces_ef.zero();
            }

            // not landed if above terrain
            if (landed && (alt_terrain_cm > 1)) {
                landed = false;
            }
        }

        // inform user if landed state has changed
        if (landed != landed_orig) {
            if (landed) {
                // get payload location again in case it has moved
                get_payload_location(payload_loc);
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SlungPayload: landed lat:%f lon:%f alt:%4.1f",
                              (double)payload_loc.lat * 1e-7,
                              (double)payload_loc.lng * 1e-7,
                              (double)payload_loc.alt * 1e-2);
            } else {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SlungPayload: liftoff");
            }
        }
    }

    // calculate forces of gravity
    Vector3f force_gravity_NED = Vector3f(0.0f, 0.0f, GRAVITY_MSS * weight_kg);

    // tension force on payload (resists gravity, drag, centripetal force)
    Vector3f tension_force_NED;

    // tension ratio to smooth transition from line being taut to slack
    tension_ratio = 0;

    // calculate drag force (0.5 * drag_coef * air_density * velocity^2 * surface area)
    Vector3f force_drag_NED;
    Vector3f velocity_air_NED = velocity_NED;
    if (!landed) {
        velocity_air_NED -= wind_ef;
    }
    if (drag_coef > 0 && !velocity_air_NED.is_zero()) {
        const float air_density = 1.225;    // 1.225 kg/m^3 (standard sea-level density)
        const float surface_area_m2 = 0.07; // 30cm diameter sphere
        const float drag_force = 0.5 * drag_coef * air_density * velocity_air_NED.length_squared() * surface_area_m2;
        force_drag_NED = -velocity_air_NED.normalized() * drag_force;
    }

    // sanity check payload distance from vehicle and calculate tension force
    if (is_positive(payload_to_veh_length)) {

        // calculate unit vector from payload to vehicle
        const Vector3f payload_to_veh_norm = payload_to_veh.normalized().tofloat();

        // ensure payload is no more than line_length from vehicle
        if (payload_to_veh_length > line_length) {
            payload_to_veh *= (line_length / payload_to_veh_length);
            position_NED = veh_pos - payload_to_veh;
        }

        // calculate tension ratio as value between 0 and 1
        // tension ratio is 0 when payload-to-vehicle distance is 10cm less than line length
        // tension ratio is 1 when payload-to-vehicle distance is equal to line length
        tension_ratio = constrain_float(1.0 - (line_length - payload_to_veh_length) * 10, 0, 1);

        // calculate tension forces when line is taut
        if (is_positive(tension_ratio)) {

            // tension resists gravity if vehicle is above payload
            if (is_negative(payload_to_veh_norm.z)) {
                tension_force_NED += -force_gravity_NED.projected(payload_to_veh_norm);
            }

            // calculate tension force resulting from velocity difference between vehicle and payload
            // use time constant to convert velocity to acceleration
            const float velocity_to_accel_TC = 2.0;
            Vector3f velocity_diff_NED = (veh_vel_ef - velocity_NED).projected(payload_to_veh_norm);

            // add to tension force if the vehicle is moving faster than the payload
            if (vectors_same_direction(velocity_diff_NED, payload_to_veh_norm)) {
                tension_force_NED += velocity_diff_NED / velocity_to_accel_TC * weight_kg;
            }

            // tension force resisting payload drag
            tension_force_NED += -force_drag_NED.projected(payload_to_veh_norm);

            // calculate centripetal force
            const Vector3f velocity_parallel = velocity_NED.projected(payload_to_veh_norm);
            const Vector3f velocity_perpendicular = velocity_NED - velocity_parallel;
            const float tension_force_centripetal = velocity_perpendicular.length_squared() * weight_kg / line_length;
            const Vector3f tension_force_centripetal_NED = payload_to_veh_norm * tension_force_centripetal;

            // add centripetal force to tension force
            tension_force_NED += tension_force_centripetal_NED;

            // scale tension force by tension ratio
            tension_force_NED *= tension_ratio;
        }
    }

    // force on vehicle is opposite to tension force on payload
    veh_forces_ef = -tension_force_NED;

    // convert force to acceleration (f=m*a => a=f/m)
    accel_NED = (force_gravity_NED + force_drag_NED + tension_force_NED) / weight_kg;

    // if slung payload is landed we zero out downward (e.g positive) acceleration
    if (landed) {
        accel_NED.z = MIN(accel_NED.z, 0);
        // should probably zero out forces_ef vertical component as well?
    }
}

// returns true if the two vectors point in the same direction, false if perpendicular or opposite
bool SlungPayloadSim::vectors_same_direction(const Vector3f& v1, const Vector3f& v2) const
{
    // check both vectors are non-zero
    if (v1.is_zero() || v2.is_zero()) {
        return false;
    }
    return v1.dot(v2) > 0;
}

#endif
