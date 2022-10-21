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
 *   Created: 08.05.2020
 *    Author: Dipl.Ing.(FH) WillyZehnder
 */
/// @file    AP_Relocate_Mission.cpp
/// @brief   translates and rotates Missions according to the location and heading where Mode-AUTO has been switched on

/*
 *   The AP_Relocate_Mission library
 *   - memorizes the Location where Mode-AUTO has been switched on (Basepoint)
 *   - moves the individual Waypoints according to Base-Point and current switch setting
 */

#include <AP_Math/AP_Math.h>
#include <AP_Common/Location.h>
#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include "AP_Relocate_Mission.h"

AP_Relocate_Mission *AP_Relocate_Mission::_singleton;

// constructor
AP_Relocate_Mission::AP_Relocate_Mission(void) {
    _singleton = this;
};

void AP_Relocate_Mission::memorize_basepoint(void)
{
    ModeReason mode_reason = AP::vehicle()->get_control_mode_reason();

    if ((mode_reason != ModeReason::RC_COMMAND) && (mode_reason != ModeReason::GCS_COMMAND)) {
        // just let the mission like it is (e.g. when soaring)
        return;
    }

// no translation of the Mission within a radius of XX m around Homepoint
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
  #define NO_TRANSLATION_RADIUS 50
#elif APM_BUILD_TYPE(APM_BUILD_ArduCopter)
  #define NO_TRANSLATION_RADIUS 10
#else
  #define NO_TRANSLATION_RADIUS 5
#endif

    _no_translation_radius = NO_TRANSLATION_RADIUS;

    RC_Channel *c = rc().find_channel_for_option(RC_Channel::AUX_FUNC::RELOCATE_MISSION);
    _restart_behaviour = Restart_Behaviour::RESTART_NOT_TRANSLATED;
    if (c != nullptr) {
        switch (c->get_aux_switch_pos()) {
        case RC_Channel::AuxSwitchPos::LOW:
            break;
        case RC_Channel::AuxSwitchPos::MIDDLE:
            _restart_behaviour = Restart_Behaviour::RESTART_PARALLEL_TRANSLATED;
            break;
        case RC_Channel::AuxSwitchPos::HIGH:
            _restart_behaviour = Restart_Behaviour::RESTART_ROTATED_HEADING;
            break;
        }
    }
    if (_restart_behaviour == Restart_Behaviour::RESTART_NOT_TRANSLATED) {
        _translation.do_translation = false;
        return;
    }

    _translation.calculated = false;
    _translation.do_translation = true;

    // memorize Basepoint (location of switching to AUTO)
    if (!AP::ahrs().get_location(_basepoint_loc)) { // _basepoint_loc.alt is absolute in [cm] in every case
        _translation.do_translation = false;
        return;
    }
    AP_Mission::Mission_Command tmp_command;

    // no translation if the distance to Homepoint is too small
    tmp_command.content.location = AP::ahrs().get_home();
    float tmp_distance = tmp_command.content.location.get_distance(_basepoint_loc); // in [m]
    if (tmp_distance < _no_translation_radius) {
         gcs().send_text(MAV_SEVERITY_NOTICE, "MR: distance to Home: %.1fm -> NO translation", tmp_distance);
        _translation.do_translation = false;
        return;
    }

    // get amount of rotation
    if (_restart_behaviour == Restart_Behaviour::RESTART_ROTATED_HEADING) {
        _translation.direction = AP::ahrs().yaw_sensor; // rotation via heading at Basepoint
    }
    else {
            _translation.direction = 0;
    }
    gcs().send_text(MAV_SEVERITY_INFO, "Restart of relocated Mission");
}

void AP_Relocate_Mission::set_no_translation() {
    _translation.do_translation = false;
}

void AP_Relocate_Mission::move_location(Location& loc, const uint16_t id)
{
    //  calculate and do translation/rotation
    if (_translation.do_translation) { // no translation if generally off by switch or if we are behind DO_LAND_START
        switch (id) { // only real waypoints will be translated
            case MAV_CMD_NAV_WAYPOINT:
            case MAV_CMD_NAV_SPLINE_WAYPOINT:
                // calculate parallel translation and corresponding values (just once at first Waypoint)
                if ((!_translation.calculated)&&(_restart_behaviour >= Restart_Behaviour::RESTART_PARALLEL_TRANSLATED)) {
                    _translation.calculated = true;
                    _first_wp_loc = loc; // memorize NOT translated position of very first WayPoint

                    // calculate altitude-translation
                    if (!_basepoint_loc.change_alt_frame(loc.get_alt_frame())) {
                        _translation.do_translation = false;
                        return;
                    }
                    _translation.alt = _basepoint_loc.alt - loc.alt;
                    if (_translation.alt < 0) { // allow just positive offsets for altitude
                        _translation.alt = 0;
                    }
                    gcs().send_text(MAV_SEVERITY_INFO, "MR: altitude translation: +%d m", (int)_translation.alt/100);
                    if (!AP::ahrs().get_location(loc)) { //  // set current location to target to avoid additional loop for very quick vehicles or small WP_RADIUS
                        _translation.do_translation = false;
                        return;
                    }
                // do the movement
                } else {
                    if (_restart_behaviour >= Restart_Behaviour::RESTART_PARALLEL_TRANSLATED){ // do parallel translation
                        AP_Relocate_Mission::translate_location(loc);
                    }
                    if (_restart_behaviour == Restart_Behaviour::RESTART_ROTATED_HEADING){ // do additional rotation
                        AP_Relocate_Mission::rotate_location(loc);
                    }
                }
                break;
            default:
                break;
        }
    }
}

void AP_Relocate_Mission::translate_location(Location& loc)
{
    Location untranslated_loc = loc;
    loc.lat = _basepoint_loc.lat + (loc.lat - _first_wp_loc.lat);
    loc.lng = _basepoint_loc.lng + (loc.lng - _first_wp_loc.lng) / untranslated_loc.longitude_scale(untranslated_loc.lat) * loc.longitude_scale(loc.lat); // correction of lng, based on lat-translation
    loc.alt += _translation.alt;
}

void AP_Relocate_Mission::rotate_location(Location& loc)
{
    Vector2f rot((loc.lng - _basepoint_loc.lng)*loc.longitude_scale(loc.lat), loc.lat - _basepoint_loc.lat);
    rot.rotate(radians((-0.01)*_translation.direction));
    loc.lat = _basepoint_loc.lat + rot.y;
    loc.lng = _basepoint_loc.lng + rot.x/loc.longitude_scale(loc.lat);
}

namespace AP {
    AP_Relocate_Mission &mission_relative() {
        return *AP_Relocate_Mission::get_singleton();
    }
}
