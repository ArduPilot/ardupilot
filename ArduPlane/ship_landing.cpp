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
  ship landing code for quadplanes
 */

#include "Plane.h"


/*
  is ship landing enabled
*/
bool QuadPlane::ship_landing_enabled(void) const
{
    return available() && ((options & OPTION_SHIP_LANDING) != 0);
}

/*
  init for ship landing in RTL
*/
void QuadPlane::ship_landing_RTL_init(void)
{
    ship_landing.stage = ship_landing.HOLDOFF;
    ship_landing.reached_alt = false;
    ship_landing_RTL_update();
    ship_landing.offset.zero();
}


/*
  update for ship landing in RTL
*/
void QuadPlane::ship_landing_RTL_update(void)
{
    uint32_t now = AP_HAL::millis();
    uint32_t last_update_ms = plane.g2.follow.get_last_update_ms();
    if (ship_landing.last_update_ms == last_update_ms ||
        now - last_update_ms > 30*1000U) {
        // don't update position if older than 30s or not changed
        return;
    }
    ship_landing.last_update_ms = last_update_ms;

    Location loc0;
    Vector3f vel;
    if (!plane.g2.follow.get_target_location_and_velocity_ofs_abs(loc0, vel)) {
        return;
    }
    Location loc = loc0;

    IGNORE_RETURN(plane.ahrs.set_home(loc));

    if (ship_landing.stage == ship_landing.HOLDOFF) {
        // hold loiter behind and to one side
        const float radius = plane.aparm.loiter_radius;
        const float holdoff_dist = radius*1.5;

        float heading_deg;
        plane.g2.follow.get_target_heading_deg(heading_deg);

        Vector2f ofs(-fabsf(holdoff_dist), holdoff_dist);
        ofs.rotate(radians(heading_deg));
        loc.offset(ofs.x, ofs.y);
        loc.alt += plane.g.RTL_altitude_cm;

        // we consider the alt to be reached if we get below 5m above
        // the target
        if (plane.current_loc.alt < loc.alt+500) {
            ship_landing.reached_alt = true;
        }

        int16_t throttle_in = plane.channel_throttle->get_control_in();
        if (throttle_in <= 0) {
            // go to approach stage when throttle is low, we are
            // pointing at the ship and have reached target alt.
            // Also require we are within 2.5 radius of the ship, and our heading is within 20
            // degrees of the target heading
            float target_bearing_deg = wrap_180(degrees(plane.current_loc.get_bearing(loc0)));
            float ground_bearing_deg = wrap_180(degrees(plane.ahrs.groundspeed_vector().angle()));
            const float margin = 10;
            if (ship_landing.reached_alt &&
                fabsf(wrap_180(target_bearing_deg - ground_bearing_deg)) < margin &&
                plane.current_loc.get_distance(loc0) < 2.5*holdoff_dist &&
                fabsf(wrap_180(ground_bearing_deg - heading_deg)) < 2*margin) {
                ship_landing.stage = ship_landing.APPROACH;
            }
        }
    } else if (ship_landing.stage == ship_landing.APPROACH) {
        // fly directly towards the ship, at QRTL_ALT
        loc.alt += qrtl_alt * 100;
    }

    plane.next_WP_loc = loc;
}

/*
  update xy controller for moving takeoffs and landings
 */
void QuadPlane::ship_update_xy(void)
{
    Location loc;
    Vector3f vel;
    Vector3f pos;

    if (!plane.g2.follow.get_target_location_and_velocity_ofs_abs(loc, vel)) {
        return;
    }

    if (!loc.get_vector_from_origin_NEU(pos)) {
        return;
    }

    // add in offset for takeoff position and landing repositioning
    pos += ship_landing.offset;

    pos.z = 0;
    vel *= 100;
    vel.z = 0;

    AP::logger().Write("SHXY", "TimeUS,px,py,vx,vy,ox,oy", "Qffffff",
                       AP_HAL::micros64(),
                       pos.x * 0.01f,
                       pos.y * 0.01f,
                       vel.x * 0.01f,
                       vel.y * 0.01f,
                       ship_landing.offset.x,
                       ship_landing.offset.y);

    pos_control->input_pos_vel_xy(pos, vel,
                                  wp_nav->get_default_speed_xy(),
                                  wp_nav->get_wp_acceleration(), 1);
}

/*
  get offset to ship takeoff target
*/
void QuadPlane::ship_set_takeoff_offset(void)
{
    Location loc;
    Vector3f vel;

    if (!plane.g2.follow.get_target_location_and_velocity_ofs_abs(loc, vel)) {
        ship_landing.offset.zero();
        return;
    }

    ship_landing.offset = loc.get_distance_NED(plane.current_loc);
}

/*
  return true when ship landing is active
*/
bool QuadPlane::in_ship_landing(void) const
{
    if (!ship_landing_enabled() ||
        !plane.g2.follow.enabled() ||
        !plane.g2.follow.have_target()) {
        return false;
    }
    return plane.control_mode == &plane.mode_qrtl || in_vtol_land_sequence();
}
