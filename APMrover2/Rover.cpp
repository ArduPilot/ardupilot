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
   main Rover class, containing all vehicle specific state
*/

#include "Rover.h"
#include "version.h"

Rover::Rover(void) :
    param_loader(var_info),
    channel_steer(nullptr),
    channel_throttle(nullptr),
    channel_learn(nullptr),
    DataFlash{FIRMWARE_STRING, g.log_bitmask},
    modes(&g.mode1),
    L1_controller(ahrs, nullptr),
    nav_controller(&L1_controller),
    steerController(ahrs),
    mission(ahrs,
            FUNCTOR_BIND_MEMBER(&Rover::start_command, bool, const AP_Mission::Mission_Command&),
            FUNCTOR_BIND_MEMBER(&Rover::verify_command_callback, bool, const AP_Mission::Mission_Command&),
            FUNCTOR_BIND_MEMBER(&Rover::exit_mission, void)),
    ServoRelayEvents(relay),
#if CAMERA == ENABLED
    camera(&relay),
#endif
#if MOUNT == ENABLED
    camera_mount(ahrs, current_loc),
#endif
    control_mode(INITIALISING),
    throttle(500),
#if FRSKY_TELEM_ENABLED == ENABLED
    frsky_telemetry(ahrs, battery, sonar),
#endif
    do_auto_rotation(false),
    home(ahrs.get_home()),
    G_Dt(0.02f)
{
}
