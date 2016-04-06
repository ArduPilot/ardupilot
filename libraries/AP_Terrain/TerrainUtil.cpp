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
  find nearest grid structure given a grid_info
*/
//bool AP_Terrain::find_nearest_grid_cache(const Location &loc, struct grid_cache &nearest_cache)
bool AP_Terrain::find_nearest_grid_cache(const Location &loc)
{
    bool closest_found[2][2] = {{false, false}, {false, false}};
    uint16_t cache_closest[2][2] = {{0,0},{0,0}};;
    float block_closest_dist_sq[2][2] = {{0,0},{0,0}};
    uint8_t block_closest_idx[2][2] = {{0,0},{0,0}};
    uint8_t block_closest_idy[2][2] = {{0,0},{0,0}};
    int16_t block_closest_height[2][2] = {{0,0},{0,0}};
    Vector2f block_closest_pos_diff[2][2];
    float grid_dist_x = TERRAIN_GRID_BLOCK_SIZE_X * grid_spacing;   // size (in meters) of a cache's grid
    float grid_dist_y = TERRAIN_GRID_BLOCK_SIZE_Y * grid_spacing;   // size (in meters) of a cache's grid

    // cycle through all caches we have
    for (uint16_t i=0; i<cache_size; i++) {
        // skip cache if it's grid is invalid or empty
        if (cache[i].state != GRID_CACHE_INVALID && cache[i].grid.bitmap != 0) {
            // find distance to grid's closest edge
            Location grid_sw_corner;
            grid_sw_corner.lat = cache[i].grid.lat;
            grid_sw_corner.lng = cache[i].grid.lon;
            Vector2f sw_corner_pos_diff = location_diff(loc, grid_sw_corner);
            Vector2f edge_pos_diff = sw_corner_pos_diff;
            if (edge_pos_diff.x > 0.0f) {
                edge_pos_diff.x = MAX(0.0f, edge_pos_diff.x - grid_dist_x);
            }
            if (edge_pos_diff.y > 0.0f) {
                edge_pos_diff.y = MAX(0.0f, edge_pos_diff.y - grid_dist_y);
            }
            float grid_min_dist_sq = fabsf(edge_pos_diff.x * edge_pos_diff.y);
            bool closest_maybe = false;
            if (edge_pos_diff.x <= 0.0f && edge_pos_diff.y <= 0.0f && (!closest_found[0][0] || grid_min_dist_sq < block_closest_dist_sq[0][0])) {
                closest_maybe = true;
            } else if (edge_pos_diff.x <= 0.0f && edge_pos_diff.y >= 0.0f && (!closest_found[0][1] || grid_min_dist_sq < block_closest_dist_sq[0][1])) {
                closest_maybe = true;
            } else if (edge_pos_diff.x >= 0.0f && edge_pos_diff.y <= 0.0f && (!closest_found[1][0] || grid_min_dist_sq < block_closest_dist_sq[1][0])) {
                closest_maybe = true;
            } else if (edge_pos_diff.x >= 0.0f && edge_pos_diff.y >= 0.0f && (!closest_found[1][1] || grid_min_dist_sq < block_closest_dist_sq[1][1])) {
                closest_maybe = true;
            }
            if (closest_maybe) {
                // calc distance to each block within the grid
                for (uint8_t idx=0; idx<TERRAIN_GRID_BLOCK_SIZE_X; idx++) {
                    for (uint8_t idy=0; idy<TERRAIN_GRID_BLOCK_SIZE_Y; idy++) {
                        // check block has height data
                        if (check_bitmap(cache[i].grid, idx, idy)) {
                            Vector2f block_pos_diff = sw_corner_pos_diff + Vector2f(idx*grid_spacing, idy*grid_spacing);
                            float block_dist_sq = fabsf(block_pos_diff.x * block_pos_diff.y);
                            uint8_t ax = (block_pos_diff.x < 0) ? 0:1;
                            uint8_t ay = (block_pos_diff.y < 0) ? 0:1;
                            if (!closest_found[ax][ay] || block_dist_sq < block_closest_dist_sq[ax][ay]) {
                                closest_found[ax][ay] = true;
                                cache_closest[ax][ay] = i;
                                block_closest_dist_sq[ax][ay] = block_dist_sq;
                                block_closest_idx[ax][ay] = idx;
                                block_closest_idy[ax][ay] = idy;
                                block_closest_height[ax][ay] = cache[i].grid.height[idx][idy];
                                block_closest_pos_diff[ax][ay] = block_pos_diff;
                            }
                        }
                    }
                }
            }
        }
    }

    for (uint8_t x=0; x<2; x++) {
        for (uint8_t y=0; y<2; y++) {
            ::printf("X:%d Y:%d lat:%ld lon:%ld closest-found:%d cache:%d idx:%d idy:%d height:%d\n",
                    (int)x,
                    (int)y,
                    (long)loc.lat,
                    (long)loc.lng,
                    (int)closest_found[x][y],
                    (int)cache_closest[x][y],
                    (int)block_closest_idx[x][y],
                    (int)block_closest_idy[x][y],
                    (int)block_closest_height[x][y]);
        }
    }


    // return closest cache
    if (closest_found[0][0] || closest_found[0][1] || closest_found[1][0] || closest_found[1][1]) {
        //nearest_cache = cache[closest_cache];
        return true;
    }

    // failed because all caches are empty
    return false;
}

void AP_Terrain::dump_grid_info()
{
    ::printf("----GRID INFO------------\n");
     // cycle through all grid caches we have
    for (uint16_t i=0; i<cache_size; i++) {
        // skip over invalid and empty grid blocks
        if (cache[i].state != GRID_CACHE_INVALID && cache[i].grid.bitmap != 0) {
            ::printf("Grid:%d lat:%ld lon:%ld\n",
                    (int)i,
                    (long)cache[i].grid.lat,
                    (long)cache[i].grid.lon);
        }
    }
    ::printf("-------------------------\n");
}

/*
  find nearest grid structure given a grid_info
*/
/*AP_Terrain::grid_cache &AP_Terrain::find_nearest_grid_cache(const struct grid_info &info)
{
    // cycle through all grid caches we have

        // if this grid has points closer than our closest point

            // cycle through all the bits of the grid, determine the closest populated bit and get it's height
            // record the closest point and it's height
}*/

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
