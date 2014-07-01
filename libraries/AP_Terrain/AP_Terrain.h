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

// MAVLink sends 5x5 grids
#define TERRAIN_GRID_MAVLINK_SIZE 5

// a 2k grid_block contains 9x9 of the mavlink grids
#define TERRAIN_GRID_BLOCK_MUL 9

// giving a total grid size of a grid_block of 45x45
#define TERRAIN_GRID_BLOCK_SIZE (TERRAIN_GRID_MAVLINK_SIZE*TERRAIN_GRID_BLOCK_MUL)

// number of grid_blocks in the LRU memory cache
#define TERRAIN_GRID_BLOCK_CACHE_SIZE 10

class AP_Terrain
{
public:
    AP_Terrain(AP_AHRS &_ahrs);

    // parameters
    AP_Int8  enable;
    AP_Int16 grid_spacing; // meters between grid points

    static const struct AP_Param::GroupInfo var_info[];
    
    // update terrain state. Should be called at 1Hz or more
    void update(void);

    // return terrain height in meters above sea level for a location
    // return false if not available
    bool height_amsl(const Location &loc, float &height);

private:
    // allocate the terrain subsystem data
    void allocate(void);

    /*
      a grid block is a structure in a local file containing height
      information. Each grid block is 2048 in size, to keep file IO to
      block oriented SD cards efficient
     */
    struct PACKED grid_block {
        // south west corner of block in degrees*10^7
        int32_t lat;
        int32_t lon;

        // crc of whole block, taken with crc=0
        uint16_t crc;

        // grid spacing in meters
        uint16_t spacing;

        // heights in meters over a 45x45 grid
        int16_t height[TERRAIN_GRID_BLOCK_SIZE][TERRAIN_GRID_BLOCK_SIZE];

        // bitmap of 5x5 grids filled in from GCS
        uint8_t bitmap[(TERRAIN_GRID_BLOCK_MUL*TERRAIN_GRID_BLOCK_MUL+7)/8];
    };

    /*
      grid_info is a broken down representation of a Location, giving
      the index terms for finding the right grid
     */
    struct grid_info {
        // rounded latitude/longitude in degrees. 
        int8_t lat_degrees;
        uint8_t lon_degrees;

        // lat and lon of SW corner of 45x45 grid
        int32_t grid_lat;
        int32_t grid_lon;

        // indexes into 45x45 grid. x is north, y is east
        uint8_t idx_x;
        uint8_t idx_y;

        // fraction (0..1) within grid square. x is north, y is east
        float frac_x;
        float frac_y;

        // bit number within the 9x9 grid_block bitmap
        uint8_t bitnum;
    };

    // given a location, fill a grid_info structure
    void calculate_grid_info(const Location &loc, struct grid_info &info) const;

    // find a grid
    struct grid_block &find_grid(const Location &loc, uint16_t ofs_north, uint16_t ofs_east) const;

    // reference to AHRS, so we can ask for our position,
    // heading and speed
    AP_AHRS &ahrs;

    // cache of grids in memory, LRU
    struct {
        uint32_t last_access_ms;
        struct grid_block block;
    } grid_cache[TERRAIN_GRID_BLOCK_CACHE_SIZE];

    // last time we asked for more grids
    uint32_t last_request_time_ms;

};
#endif // __AP_TERRAIN_H__
