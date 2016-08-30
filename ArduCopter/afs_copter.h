/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
#pragma once

/*
  advanced failsafe support for copter
 */

#if ADVANCED_FAILSAFE == ENABLED
#include <AP_AdvancedFailsafe/AP_AdvancedFailsafe.h>

/*
  a plane specific AP_AdvancedFailsafe class
 */
class AP_AdvancedFailsafe_Copter : public AP_AdvancedFailsafe
{
public:
    AP_AdvancedFailsafe_Copter(AP_Mission &_mission, AP_Baro &_baro, const AP_GPS &_gps, const RCMapper &_rcmap);

    // called to set all outputs to termination state
    void terminate_vehicle(void);
    
protected:
    // setup failsafe values for if FMU firmware stops running
    void setup_IO_failsafe(void);

    // return the AFS mapped control mode
    enum control_mode afs_mode(void);
};

#endif // ADVANCED_FAILSAFE

