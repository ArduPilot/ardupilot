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

#include "SIM_Ship.h"
#include "SITL.h"

#include <stdio.h>

#include "SIM_Aircraft.h"
#include <AP_HAL_SITL/SITL_State.h>

// use a spare channel for send. This is static to avoid mavlink
// header import in SIM_Ship.h
static const mavlink_channel_t mavlink_ch = (mavlink_channel_t)(MAVLINK_COMM_0+6);

using namespace SITL;

// SITL Ship parameters
const AP_Param::GroupInfo ShipSim::var_info[] = {
    AP_GROUPINFO("ENABLE",    1, ShipSim,  enable, 0),
    AP_GROUPINFO("SPEED",     2, ShipSim,  speed, 3),
    AP_GROUPINFO("PSIZE",     3, ShipSim,  path_size, 1000),
    AP_GROUPINFO("SYSID",     4, ShipSim,  sys_id, 17),
    AP_GROUPINFO("DSIZE",     5, ShipSim,  deck_size, 10),
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
    if (!valid_channel(mavlink_ch)) {
        AP_HAL::panic("Invalid mavlink channel for ShipSim");
    }
    AP_Param::setup_object_defaults(this, var_info);
}

/*
  get ground speed adjustment if we are landed on the ship
 */
Vector2f ShipSim::get_ground_speed_adjustment(const Location &loc, float &yaw_rate)
{
    if (!enable) {
        yaw_rate = 0;
        return Vector2f(0,0);
    }
    Location shiploc = home;
    shiploc.offset(ship.position.x, ship.position.y);
    if (loc.get_distance(shiploc) > deck_size) {
        yaw_rate = 0;
        return Vector2f(0,0);
    }
    Vector2f vel(ship.speed, 0);
    vel.rotate(radians(ship.heading_deg));
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
    mavlink_message_t msg;
    uint16_t len;
    uint8_t buf[300];

    const uint8_t component_id = MAV_COMP_ID_USER10;

    if (now - last_heartbeat_ms >= 1000) {
        last_heartbeat_ms = now;
        mavlink_msg_heartbeat_pack_chan(sys_id.get(),
                                        component_id,
                                        mavlink_ch,
                                        &msg,
                                        MAV_TYPE_SURFACE_BOAT,
                                        MAV_AUTOPILOT_INVALID,
                                        0,
                                        0,
                                        0);
        len = mavlink_msg_to_send_buffer(buf, &msg);
        mav_socket.send(buf, len);
    }


    /*
      send a GLOBAL_POSITION_INT messages
     */
    Location loc = home;
    loc.offset(ship.position.x, ship.position.y);

    int32_t alt;
    bool have_alt = false;

#if AP_TERRAIN_AVAILABLE
    auto terrain = AP::terrain();
    float height;
    if (terrain != nullptr && terrain->enabled() && terrain->height_amsl(loc, height, true)) {
        alt = height * 1000;
        have_alt = true;
    }
#endif
    if (!have_alt) {
        // assume home altitude
        alt = home.alt;
    }

    Vector2f vel(ship.speed, 0);
    vel.rotate(radians(ship.heading_deg));

    mavlink_msg_global_position_int_pack_chan(sys_id,
                                              component_id,
                                              mavlink_ch,
                                              &msg,
                                              now,
                                              loc.lat,
                                              loc.lng,
                                              alt,
                                              0,
                                              vel.x*100,
                                              vel.y*100,
                                              0,
                                              ship.heading_deg*100);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    if (len > 0) {
        mav_socket.send(buf, len);
    }
}
