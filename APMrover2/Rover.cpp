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

#define FORCE_VERSION_H_INCLUDE
#include "version.h"
#undef FORCE_VERSION_H_INCLUDE

Rover::Rover(void) :
    param_loader(var_info),
    channel_steer(nullptr),
    channel_throttle(nullptr),
    channel_aux(nullptr),
    channel_lateral(nullptr),
    logger{g.log_bitmask},
    modes(&g.mode1),
    nav_controller(&L1_controller),
    control_mode(&mode_initializing),
    G_Dt(0.02f)
{
}
