// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
 
#ifndef AP_RPM_PX4_PWM_H
#define AP_RPM_PX4_PWM_H

#include "AP_RPM.h"
#include "RPM_Backend.h"

class AP_RPM_PX4_PWM : public AP_RPM_Backend
{
public:
    // constructor
    AP_RPM_PX4_PWM(AP_RPM &ranger, uint8_t instance, AP_RPM::RPM_State &_state);

    // destructor
    ~AP_RPM_PX4_PWM(void);
    
    // update state
    void update(void);

private:
    int _fd = -1;
    uint64_t _last_timestamp = 0;
};

#endif // AP_RPM_PX4_PWM_H
