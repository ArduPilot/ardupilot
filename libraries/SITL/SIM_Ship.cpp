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
  simulate ship takeoff/landing
*/

#include "SIM_config.h"

#if AP_SIM_SHIP_ENABLED

#include "SIM_Ship.h"

#include "SITL.h"

#include <stdio.h>

#include "SIM_Aircraft.h"
#include <AP_HAL_SITL/SITL_State.h>
#include <AP_Terrain/AP_Terrain.h>

using namespace SITL;

// SITL Ship parameters
const AP_Param::GroupInfo ShipSim::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Ship landing Enable
    // @Description: Enable ship landing simulation
    // @Values: 0:Disable,1:Enabled
    AP_GROUPINFO("ENABLE",    1, ShipSim,  enable, 0),
    // @Param: SPEED
    // @DisplayName: Ship Speed
    // @Description: Speed of the ship
    // @Units: m/s
    AP_GROUPINFO("SPEED",     2, ShipSim,  speed, 3),
    // @Param: PSIZE
    // @DisplayName: Path Size
    // @Description: Diameter of the circle the ship is traveling on
    // @Units: m
    AP_GROUPINFO("PSIZE",     3, ShipSim,  path_size, 1000),
    // @Param: SYSID
    // @DisplayName: System ID
    // @Description: System ID of the ship
    // @Range: 1 255
    AP_GROUPINFO("SYSID",     4, ShipSim,  sys_id, 17),
    // @Param: DSIZE
    // @DisplayName: Deck Size
    // @Description: Size of the ship's deck
    // @Units: m
    AP_GROUPINFO("DSIZE",     5, ShipSim,  deck_size, 10),
    // @Param: OFS
    // @DisplayName: Ship landing pad offset
    // @Description: Defines the offset of the ship's landing pad w.r.t. the ship's origin, i.e. where the beacon is placed on the ship
    // @Units: m
    // @Vector3Parameter: 1
    AP_GROUPINFO("OFS",       7, ShipSim,  offset, 0),
    AP_GROUPEND
};

/*
  update a simulated vehicle
 */
void Ship::update(float delta_t)
{
    // acceletate over time to keep EKF happy
    const float max_accel = 3.0;
    const float dspeed_max = max_accel * delta_t;
    speed = constrain_float(sim->speed.get(), speed-dspeed_max, speed+dspeed_max);

    // calculate how far around the circle we go
    float circumference = M_PI * sim->path_size.get();
    float dist = delta_t * speed;
    float dangle = (dist / circumference) * 360.0;

    if (delta_t > 0) {
        yaw_rate = radians(dangle) / delta_t;
    }
    heading_deg += dangle;
    heading_deg = wrap_360(heading_deg);

    Vector2f dpos(dist, 0);
    dpos.rotate(radians(heading_deg));

    position += dpos;
}

ShipSim::ShipSim()
{
    AP_Param::setup_object_defaults(this, var_info);
}

/*
  get the location of the ship
 */
bool ShipSim::get_location(Location &loc) const
{
    if (!enable) {
        return false;
    }
    loc = home;
    loc.offset(ship.position.x, ship.position.y);
    return true;
}

/*
  get ground speed adjustment if we are landed on the ship
 */
Vector2f ShipSim::get_ground_speed_adjustment(const Location &loc, float &yaw_rate)
{
    Location shiploc;
    if (!get_location(shiploc)) {
        yaw_rate = 0;
        return Vector2f(0,0);
    }
    if (loc.get_distance(shiploc) > deck_size) {
        yaw_rate = 0;
        return Vector2f(0,0);
    }

    // find center of the circle that the ship is on
    Location center = shiploc;
    const float path_radius = path_size.get()*0.5;
    center.offset_bearing(ship.heading_deg+(ship.yaw_rate>0?90:-90), path_radius);

    // scale speed for ratio of distances
    const float p = center.get_distance(loc) / path_radius;
    const float scaled_speed = ship.speed * p;

    // work out how far around the circle ahead or behind we are for
    // rotating velocity
    const float bearing1 = center.get_bearing(loc);
    const float bearing2 = center.get_bearing(shiploc);
    const float heading = ship.heading_deg + degrees(bearing1-bearing2);

    Vector2f vel(scaled_speed, 0);
    vel.rotate(radians(heading));
    yaw_rate = ship.yaw_rate;
    return vel;
}

/*
  update the ShipSim peripheral state
*/
void ShipSim::update(void)
{
    if (!enable) {
        return;
    }

    auto *sitl = AP::sitl();
    uint32_t now_us = AP_HAL::micros();

    if (!initialised) {
        home = sitl->state.home;
        if (home.lat == 0 && home.lng == 0) {
            return;
        }
        const Vector3f &ofs = offset.get();
        home.offset(ofs.x, ofs.y);
        home.alt -= ofs.z*100;

        initialised = true;
        ::printf("ShipSim home %f %f\n", home.lat*1.0e-7, home.lng*1.0e-7);
        ship.sim = this;
        last_update_us = now_us;
        last_report_ms = AP_HAL::millis();
    }

    float dt = (now_us - last_update_us)*1.0e-6;
    last_update_us = now_us;

    ship.update(dt);

    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_report_ms >= reporting_period_ms) {
        last_report_ms = now_ms;
        send_report();
    }
}

/*
  send a report to the vehicle control code over MAVLink
*/
void ShipSim::send_report(void)
{
    if (!mavlink_connected && mav_socket.connect(target_address, target_port)) {
        ::printf("ShipSim connected to %s:%u\n", target_address, (unsigned)target_port);
        mavlink_connected = true;
    }
    if (!mavlink_connected) {
        return;
    }

    uint32_t now = AP_HAL::millis();

    const uint8_t component_id = MAV_COMP_ID_USER10;

    if (now - last_heartbeat_ms >= 1000) {
        last_heartbeat_ms = now;

        const mavlink_heartbeat_t heartbeat{
        custom_mode: 0,
        type : MAV_TYPE_SURFACE_BOAT,
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


    /*
      send a GLOBAL_POSITION_INT messages
     */
    Location loc;
    if (!get_location(loc)) {
        return;
    }

    int32_t alt_mm = home.alt * 10;  // assume home altitude

#if AP_TERRAIN_AVAILABLE
    auto terrain = AP::terrain();
    float height;
    if (terrain != nullptr && terrain->enabled() && terrain->height_amsl(loc, height, false)) {
        alt_mm = height * 1000;
    }
#endif

    {  // send position
        Vector2f vel(ship.speed, 0);
        vel.rotate(radians(ship.heading_deg));

        const mavlink_global_position_int_t global_position_int{
        time_boot_ms: now,
        lat: loc.lat,
        lon: loc.lng,
        alt: alt_mm,
        relative_alt: 0,
        vx: int16_t(vel.x*100),
        vy: int16_t(vel.y*100),
        vz: 0,
        hdg: uint16_t(ship.heading_deg*100)
        };
        mavlink_message_t msg;
        mavlink_msg_global_position_int_encode_status(
            sys_id,
            component_id,
            &mav_status,
            &msg,
            &global_position_int);
        uint8_t buf[300];
        const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        if (len > 0) {
            mav_socket.send(buf, len);
        }
    }

    { // also set ATTITUDE so MissionPlanner can display ship orientation
        const mavlink_attitude_t attitude{
        time_boot_ms: now,
        roll: 0,
        pitch: 0,
        yaw: float(radians(ship.heading_deg)),
        rollspeed: 0,
        pitchspeed: 0,
        yawspeed: ship.yaw_rate
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

#endif  // AP_SIM_SHIP_ENABLED
