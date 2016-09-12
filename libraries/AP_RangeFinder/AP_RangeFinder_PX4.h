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
#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

class AP_RangeFinder_PX4 : public AP_RangeFinder_Backend
{
public:
    // constructor
    AP_RangeFinder_PX4(RangeFinder &ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state);

    // destructor
    ~AP_RangeFinder_PX4(void);
    
    // static detection function
    static bool detect(RangeFinder &ranger, uint8_t instance);

    // update state
    void update(void);

private:
    int _fd;
    uint64_t _last_timestamp;

    int16_t _last_max_distance_cm;
    int16_t _last_min_distance_cm;

    // we need to keep track of how many PX4 drivers have been loaded
    // so we can open the right device filename
    static uint8_t num_px4_instances;

    // try to open the PX4 driver and return its fd
    static int open_driver(void);
};
