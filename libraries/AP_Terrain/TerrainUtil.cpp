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
/*
  handle disk IO for terrain code
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_Terrain.h"

#if AP_TERRAIN_AVAILABLE

#include <assert.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>

extern const AP_HAL::HAL& hal;

/*
  calculate bit number in grid_block bitmap. This corresponds to a
  bit representing a 4x4 mavlink transmitted block
*/
uint8_t AP_Terrain::grid_bitnum(uint8_t idx_x, uint8_t idx_y)
{    
    ASSERT_RANGE(idx_x,0,27);
    ASSERT_RANGE(idx_y,0,31);
    uint8_t subgrid_x = idx_x / TERRAIN_GRID_MAVLINK_SIZE;
    uint8_t subgrid_y = idx_y / TERRAIN_GRID_MAVLINK_SIZE;
    ASSERT_RANGE(subgrid_x,0,TERRAIN_GRID_BLOCK_MUL_X-1);
    ASSERT_RANGE(subgrid_y,0,TERRAIN_GRID_BLOCK_MUL_Y-1);
    return subgrid_y + TERRAIN_GRID_BLOCK_MUL_Y*subgrid_x;
}

/*
  given a grid_info check that a given idx_x/idx_y is available (set
  in the bitmap)
 */
bool AP_Terrain::check_bitmap(const struct grid_block &grid, uint8_t idx_x, uint8_t idx_y)
{
    uint8_t bitnum = grid_bitnum(idx_x, idx_y);
    return (grid.bitmap & (((uint64_t)1U)<<bitnum)) != 0;
}

/*
  given a location, calculate the 32x28 grid SW corner, plus the
  grid indices
*/
void AP_Terrain::calculate_grid_info(const Location &loc, struct grid_info &info) const
{
    // grids start on integer degrees. This makes storing terrain data
    // on the SD card a bit easier
    info.lat_degrees = (loc.lat<0?(loc.lat-9999999L):loc.lat) / (10*1000*1000L);
    info.lon_degrees = (loc.lng<0?(loc.lng-9999999L):loc.lng) / (10*1000*1000L);

    // create reference position for this rounded degree position
    Location ref;
    ref.lat = info.lat_degrees*10*1000*1000L;
    ref.lng = info.lon_degrees*10*1000*1000L;

    // find offset from reference
    Vector2f offset = location_diff(ref, loc);

    // get indices in terms of grid_spacing elements
    uint32_t idx_x = offset.x / grid_spacing;
    uint32_t idx_y = offset.y / grid_spacing;

    // find indexes into 32*28 grids for this degree reference. Note
    // the use of TERRAIN_GRID_BLOCK_SPACING_{X,Y} which gives a one square
    // overlap between grids
    info.grid_idx_x = idx_x / TERRAIN_GRID_BLOCK_SPACING_X;
    info.grid_idx_y = idx_y / TERRAIN_GRID_BLOCK_SPACING_Y;

    // find the indices within the 32*28 grid
    info.idx_x = idx_x % TERRAIN_GRID_BLOCK_SPACING_X;
    info.idx_y = idx_y % TERRAIN_GRID_BLOCK_SPACING_Y;

    // find the fraction (0..1) within the square
    info.frac_x = (offset.x - idx_x * grid_spacing) / grid_spacing;
    info.frac_y = (offset.y - idx_y * grid_spacing) / grid_spacing;

    // calculate lat/lon of SW corner of 32*28 grid_block
    location_offset(ref, 
                    info.grid_idx_x * TERRAIN_GRID_BLOCK_SPACING_X * (float)grid_spacing,
                    info.grid_idx_y * TERRAIN_GRID_BLOCK_SPACING_Y * (float)grid_spacing);
    info.grid_lat = ref.lat;
    info.grid_lon = ref.lng;

    ASSERT_RANGE(info.idx_x,0,TERRAIN_GRID_BLOCK_SPACING_X-1);
    ASSERT_RANGE(info.idx_y,0,TERRAIN_GRID_BLOCK_SPACING_Y-1);
    ASSERT_RANGE(info.frac_x,0,1);
    ASSERT_RANGE(info.frac_y,0,1);
}


/*
  find a grid structure given a grid_info
 */
AP_Terrain::grid_cache &AP_Terrain::find_grid_cache(const struct grid_info &info)
{
    uint16_t oldest_i = 0;

    // see if we have that grid
    for (uint16_t i=0; i<cache_size; i++) {
        if (cache[i].grid.lat == info.grid_lat && 
            cache[i].grid.lon == info.grid_lon &&
            cache[i].grid.spacing == grid_spacing) {
            cache[i].last_access_ms = AP_HAL::millis();
            return cache[i];
        }
        if (cache[i].last_access_ms < cache[oldest_i].last_access_ms) {
            oldest_i = i;
        }
    }

    // Not found. Use the oldest grid and make it this grid,
    // initially unpopulated
    struct grid_cache &grid = cache[oldest_i];
    memset(&grid, 0, sizeof(grid));

    grid.grid.lat = info.grid_lat;
    grid.grid.lon = info.grid_lon;
    grid.grid.spacing = grid_spacing;
    grid.grid.grid_idx_x = info.grid_idx_x;
    grid.grid.grid_idx_y = info.grid_idx_y;
    grid.grid.lat_degrees = info.lat_degrees;
    grid.grid.lon_degrees = info.lon_degrees;
    grid.grid.version = TERRAIN_GRID_FORMAT_VERSION;
    grid.last_access_ms = AP_HAL::millis();

    // mark as waiting for disk read
    grid.state = GRID_CACHE_DISKWAIT;

    return grid;
}

/*
  find cache index of disk_block
 */
int16_t AP_Terrain::find_io_idx(enum GridCacheState state)
{
    // try first with given state
    for (uint16_t i=0; i<cache_size; i++) {
        if (disk_block.block.lat == cache[i].grid.lat &&
            disk_block.block.lon == cache[i].grid.lon && 
            cache[i].state == state) {
            return i;
        }
    }    
    // then any state
    for (uint16_t i=0; i<cache_size; i++) {
        if (disk_block.block.lat == cache[i].grid.lat &&
            disk_block.block.lon == cache[i].grid.lon) {
            return i;
        }
    }    
    return -1;
}

/*
  get CRC for a block
 */
uint16_t AP_Terrain::get_block_crc(struct grid_block &block)
{
    uint16_t saved_crc = block.crc;
    block.crc = 0;
    uint16_t ret = crc16_ccitt((const uint8_t *)&block, sizeof(block), 0);
    block.crc = saved_crc;
    return ret;
}

#endif // AP_TERRAIN_AVAILABLE
