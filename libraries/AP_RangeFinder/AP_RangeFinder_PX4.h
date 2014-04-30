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
 
#ifndef AP_RangeFinder_PX4_H
#define AP_RangeFinder_PX4_H

#include "RangeFinder.h"

class AP_RangeFinder_PX4 : public RangeFinder
{
public:
    // constructor
    AP_RangeFinder_PX4(FilterInt16 *filter);
    
    // initialize all the range finder devices
    bool init(void);
    
    bool take_reading(void);
    
    void accumulate(void);
    
    // read value from primary sensor and return distance in cm
    int16_t read();
    
    // return the number of compass instances
    uint8_t get_count(void) const { return _num_instances; }
private:
    uint8_t _get_primary(void) const;
    uint8_t _num_instances;
    int _range_fd[RANGEFINDER_MAX_INSTANCES];
    float _sum[RANGEFINDER_MAX_INSTANCES];
    uint32_t _count[RANGEFINDER_MAX_INSTANCES];
    uint64_t _last_timestamp[RANGEFINDER_MAX_INSTANCES];
};

#endif // AP_RangeFinder_PX4_H
