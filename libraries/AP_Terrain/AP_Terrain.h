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

#ifndef __AP_TERRAIN_H__
#define __AP_TERRAIN_H__

#include <AP_Common.h>
#include <AP_HAL.h>
#include <AP_Param.h>
#include <AP_AHRS.h>

class AP_Terrain
{
public:
    AP_Terrain(const AP_AHRS &_ahrs);

    // parameters
    AP_Int8  enable;
    AP_Int16 grid_spacing; // meters between grid points
    AP_Int16 grid_width;   // number of grid points across

    static const struct AP_Param::GroupInfo var_info[];
    
    // update terrain state. Should be called at 1Hz or more
    void update(void);

    // return terrain height in meters above sea level for a location
    // return false if not available
    bool height_amsl(const Location &loc);
    
private:
    // allocate the terrain subsystem data
    void allocate(void);

    // reference to AHRS, so we can ask for our position,
    // heading and speed
    const AP_AHRS &ahrs;

    // the grid spacing for the current data
    uint16_t last_grid_spacing;

    // number of grids in array
    uint16_t grids_allocated;

    // a single 5x5 grid, matching a single MAVLink TERRAIN_DATA message
    struct grid {
        int32_t lat;
        int32_t lon;
        int16_t height[5][5];
        bool valid:1;
    } grid;

    // current grids
    struct grid *grids;

    // last time we asked for more grids
    uint32_t last_request_time_ms;

};
#endif // __AP_TERRAIN_H__
