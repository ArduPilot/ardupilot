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

#if HAL_OS_POSIX_IO
#define HAVE_AP_TERRAIN 1
#else
#define HAVE_AP_TERRAIN 0
#endif

#if HAVE_AP_TERRAIN

#include <AP_Param.h>
#include <AP_AHRS.h>

// MAVLink sends 4x4 grids
#define TERRAIN_GRID_MAVLINK_SIZE 4

// a 2k grid_block on disk contains 8x7 of the mavlink grids.  Each
// grid block overlaps by one with its neighbour. This ensures that
// the altitude at any point can be calculated from a single grid
// block
#define TERRAIN_GRID_BLOCK_MUL_X 7
#define TERRAIN_GRID_BLOCK_MUL_Y 8

// this is the spacing between 32x28 grid blocks, in grid_spacing units
#define TERRAIN_GRID_BLOCK_SPACING_X ((TERRAIN_GRID_BLOCK_MUL_X-1)*TERRAIN_GRID_MAVLINK_SIZE)
#define TERRAIN_GRID_BLOCK_SPACING_Y ((TERRAIN_GRID_BLOCK_MUL_Y-1)*TERRAIN_GRID_MAVLINK_SIZE)

// giving a total grid size of a disk grid_block of 32x28
#define TERRAIN_GRID_BLOCK_SIZE_X (TERRAIN_GRID_MAVLINK_SIZE*TERRAIN_GRID_BLOCK_MUL_X)
#define TERRAIN_GRID_BLOCK_SIZE_Y (TERRAIN_GRID_MAVLINK_SIZE*TERRAIN_GRID_BLOCK_MUL_Y)

// number of grid_blocks in the LRU memory cache
#define TERRAIN_GRID_BLOCK_CACHE_SIZE 12

// format of grid on disk
#define TERRAIN_GRID_FORMAT_VERSION 1

/*
  Data conventions in this library:

  array[x][y]: x is increasing north, y is increasing east
  array[x]:    low order bits increase east first
  bitmap:      low order bits increase east first
 */

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

    // send any pending terrain request message
    void send_request(mavlink_channel_t chan);

    // handle terrain data from GCS
    void handle_data(mavlink_message_t *msg);

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
        // crc of whole block, taken with crc=0
        uint16_t crc;

        // format version number
        uint16_t version;

        // grid spacing in meters
        uint16_t spacing;

        // heights in meters over a 32*28 grid
        int16_t height[TERRAIN_GRID_BLOCK_SIZE_X][TERRAIN_GRID_BLOCK_SIZE_Y];

        // south west corner of block in degrees*10^7
        int32_t lat;
        int32_t lon;

        // bitmap of 4x4 grids filled in from GCS (56 bits are used)
        uint64_t bitmap;
    };

    /*
      grid_block for disk IO, aligned on 2048 byte boundaries
     */
    union grid_io_block {
        struct grid_block grid;
        uint8_t buffer[2048];
    };

    /*
      a grid_block plus some meta data used for requesting new blocks
     */
    struct grid_cache {
        struct grid_block grid;

        // the last time access was requested to this block, used for LRU
        uint32_t last_access_ms;
    };

    /*
      grid_info is a broken down representation of a Location, giving
      the index terms for finding the right grid
     */
    struct grid_info {
        // rounded latitude/longitude in degrees. 
        int8_t lat_degrees;
        int16_t lon_degrees;

        // lat and lon of SW corner of this 32*28 grid, *10^7 degrees
        int32_t grid_lat;
        int32_t grid_lon;

        // indexes into 32x28 grid
        uint8_t idx_x; // north (0..27)
        uint8_t idx_y; // east  (0..31)

        // fraction within the grid square
        float frac_x; // north (0..1)
        float frac_y; // east  (0..1)
    };

    // given a location, fill a grid_info structure
    void calculate_grid_info(const Location &loc, struct grid_info &info) const;

    /*
      find a grid structure given a grid_info
    */
    struct grid_cache &find_grid(const struct grid_info &info);

    /*
      calculate bit number in grid_block bitmap. This corresponds to a
      bit representing a 4x4 mavlink transmitted block
    */
    uint8_t grid_bitnum(uint8_t idx_x, uint8_t idx_y);

    /*
      given a grid_info check that a given idx_x/idx_y is available (set
      in the bitmap)
    */
    bool check_bitmap(const struct grid_block &grid, uint8_t idx_x, uint8_t idx_y);

    /*
      request any missing 4x4 grids from a block
    */
    bool request_missing(mavlink_channel_t chan, const struct grid_info &info);

    // reference to AHRS, so we can ask for our position,
    // heading and speed
    AP_AHRS &ahrs;

    // cache of grids in memory, LRU
    struct grid_cache cache[TERRAIN_GRID_BLOCK_CACHE_SIZE];

    // last time we asked for more grids
    uint32_t last_request_time_ms;

};
#endif // HAVE_AP_TERRAIN
#endif // __AP_TERRAIN_H__
