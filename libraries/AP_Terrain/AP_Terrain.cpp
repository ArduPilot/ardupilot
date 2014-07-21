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

#include <AP_HAL.h>
#include <AP_Common.h>
#include <AP_Math.h>
#include <GCS_MAVLink.h>
#include <GCS.h>
#include "AP_Terrain.h"
#include <assert.h>
#include <stdio.h>

#if HAVE_AP_TERRAIN

#define TERRAIN_DEBUG 0

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_Terrain::var_info[] PROGMEM = {
    // @Param: ENABLE
    // @DisplayName: Terrain following enable
    // @Description: enable terrain following
    // @Values: 0:Disable,1:Enable
    AP_GROUPINFO("ENABLE",    0, AP_Terrain, enable, 0),

    // @Param: SPACING
    // @DisplayName: Terrain grid spacing
    // @Description: distance between terrain grid points in meters
    // @Units: meters
    // @Increment: 1
    AP_GROUPINFO("SPACING",   1, AP_Terrain, grid_spacing, 100),

    AP_GROUPEND
};

// constructor
AP_Terrain::AP_Terrain(AP_AHRS &_ahrs) :
    ahrs(_ahrs),
    last_request_time_ms(0)
{
    AP_Param::setup_object_defaults(this, var_info);
}

#if TERRAIN_DEBUG
#define ASSERT_RANGE(v,minv,maxv) assert((v)<=(maxv)&&(v)>=(minv))
#else
#define ASSERT_RANGE(v,minv,maxv)
#endif

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
    uint16_t grid_idx_x = idx_x / TERRAIN_GRID_BLOCK_SPACING_X;
    uint16_t grid_idx_y = idx_y / TERRAIN_GRID_BLOCK_SPACING_Y;

    // find the indices within the 32*28 grid
    info.idx_x = idx_x % TERRAIN_GRID_BLOCK_SPACING_X;
    info.idx_y = idx_y % TERRAIN_GRID_BLOCK_SPACING_Y;

    // find the fraction (0..1) within the square
    info.frac_x = (offset.x - idx_x * grid_spacing) / grid_spacing;
    info.frac_y = (offset.y - idx_y * grid_spacing) / grid_spacing;

    // calculate lat/lon of SW corner of 32*28 grid_block
    location_offset(ref, 
                    grid_idx_x * TERRAIN_GRID_BLOCK_SPACING_X * (float)grid_spacing,
                    grid_idx_y * TERRAIN_GRID_BLOCK_SPACING_Y * (float)grid_spacing);
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
AP_Terrain::grid_cache &AP_Terrain::find_grid(const struct grid_info &info)
{
    uint16_t oldest_i = 0;

    // see if we have that grid
    for (uint16_t i=0; i<TERRAIN_GRID_BLOCK_CACHE_SIZE; i++) {
        if (cache[i].grid.lat == info.grid_lat && 
            cache[i].grid.lon == info.grid_lon) {
            cache[i].last_access_ms = hal.scheduler->millis();
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
    grid.last_access_ms = hal.scheduler->millis();

    return grid;
}

/*
  return terrain height in meters above average sea level (WGS84) for
  a given position
 */
bool AP_Terrain::height_amsl(const Location &loc, float &height)
{
    if (!enable) {
        return false;
    }

    struct grid_info info;

    calculate_grid_info(loc, info);

    // find the grid
    const struct grid_block &grid = find_grid(info).grid;

    /*
      note that we rely on the one square overlap to ensure these
      calculations don't go past the end of the arrays
     */
    ASSERT_RANGE(info.idx_x, 0, TERRAIN_GRID_BLOCK_SIZE_X-2);
    ASSERT_RANGE(info.idx_y, 0, TERRAIN_GRID_BLOCK_SIZE_Y-2);


    // check we have all 4 required heights
    if (!check_bitmap(grid, info.idx_x,   info.idx_y) ||
        !check_bitmap(grid, info.idx_x,   info.idx_y+1) ||
        !check_bitmap(grid, info.idx_x+1, info.idx_y) ||
        !check_bitmap(grid, info.idx_x+1, info.idx_y+1)) {
        return false;
    }

    // hXY are the heights of the 4 surrounding grid points
    int16_t h00, h01, h10, h11;

    h00 = grid.height[info.idx_x+0][info.idx_y+0];
    h01 = grid.height[info.idx_x+0][info.idx_y+1];
    h10 = grid.height[info.idx_x+1][info.idx_y+0];
    h11 = grid.height[info.idx_x+1][info.idx_y+1];

    float avg1 = (1.0f-info.frac_x) * h00  + info.frac_x * h10;
    float avg2 = (1.0f-info.frac_x) * h01  + info.frac_x * h11;
    float avg  = (1.0f-info.frac_y) * avg1 + info.frac_y * avg2;

    height = avg;
    return true;
}

/*
  request any missing 4x4 grids from a block
 */
bool AP_Terrain::request_missing(mavlink_channel_t chan, const struct grid_info &info)
{
    // find the grid
    struct grid_block &grid = find_grid(info).grid;

    // see if it is fully populated
    if ((grid.bitmap & bitmap_mask) == bitmap_mask) {
        // it is fully populated, nothing to do
        return false;
    }

    /*
      ask the GCS to send a set of 4x4 grids
     */
    mavlink_msg_terrain_request_send(chan, grid.lat, grid.lon, grid_spacing, bitmap_mask & ~grid.bitmap);
    last_request_time_ms = hal.scheduler->millis();

    return true;
}

/*
  send any pending terrain request to the GCS
 */
void AP_Terrain::send_request(mavlink_channel_t chan)
{
    if (enable == 0) {
        // not enabled
        return;
    }

    // did we request recently?
    if (hal.scheduler->millis() - last_request_time_ms < 2000) {
        // too soon to request again
        return;
    }

    Location loc;
    if (!ahrs.get_position(loc)) {
        // we don't know where we are
        return;
    }

    // request any missing 4x4 blocks in the current grid
    struct grid_info info;
    calculate_grid_info(loc, info);

    if (request_missing(chan, info)) {
        return;
    }

    // also request a larger set of up to 9 grids
    for (int8_t x=-1; x<=1; x++) {
        for (int8_t y=-1; y<=1; y++) {
            Location loc2 = loc;
            location_offset(loc2, 
                            x*TERRAIN_GRID_BLOCK_SIZE_X*0.7f*grid_spacing,
                            y*TERRAIN_GRID_BLOCK_SIZE_Y*0.7f*grid_spacing);
            struct grid_info info2;
            calculate_grid_info(loc2, info2);            
            if (request_missing(chan, info2)) {
                return;
            }
        }
    }

    // request the current loc last to ensure it has highest last
    // access time
    if (request_missing(chan, info)) {
        return;
    }
}

/* 
   handle terrain data from GCS
 */
void AP_Terrain::handle_data(mavlink_message_t *msg)
{
        mavlink_terrain_data_t packet;
        mavlink_msg_terrain_data_decode(msg, &packet);

        uint16_t i;
        for (i=0; i<TERRAIN_GRID_BLOCK_CACHE_SIZE; i++) {
            if (cache[i].grid.lat == packet.lat && 
                cache[i].grid.lon == packet.lon && 
                cache[i].grid.spacing == packet.grid_spacing &&
                packet.gridbit < 56) {
                break;
            }
        }
        if (i == TERRAIN_GRID_BLOCK_CACHE_SIZE) {
            // we don't have that grid, ignore data
            return;
        }
        struct grid_cache &gcache = cache[i];
        struct grid_block &grid = gcache.grid;
        uint8_t idx_x = (packet.gridbit / TERRAIN_GRID_BLOCK_MUL_Y) * TERRAIN_GRID_MAVLINK_SIZE;
        uint8_t idx_y = (packet.gridbit % TERRAIN_GRID_BLOCK_MUL_Y) * TERRAIN_GRID_MAVLINK_SIZE;
        ASSERT_RANGE(idx_x,0,(TERRAIN_GRID_BLOCK_MUL_X-1)*TERRAIN_GRID_MAVLINK_SIZE);
        ASSERT_RANGE(idx_y,0,(TERRAIN_GRID_BLOCK_MUL_Y-1)*TERRAIN_GRID_MAVLINK_SIZE);
        for (uint8_t x=0; x<TERRAIN_GRID_MAVLINK_SIZE; x++) {
            for (uint8_t y=0; y<TERRAIN_GRID_MAVLINK_SIZE; y++) {
                grid.height[idx_x+x][idx_y+y] = packet.data[x*TERRAIN_GRID_MAVLINK_SIZE+y];
                ASSERT_RANGE(grid.height[idx_x+x][idx_y+y], 1, 20000);
            }
        }
        gcache.grid.bitmap |= ((uint64_t)1) << packet.gridbit;
#if TERRAIN_DEBUG
        hal.console->printf("Filled bit %u idx_x=%u idx_y=%u\n", 
                            (unsigned)packet.gridbit, (unsigned)idx_x, (unsigned)idx_y);
        if (gcache.grid.bitmap == bitmap_mask) {
            hal.console->printf("--lat=%12.7f --lon=%12.7f %u\n",
                                grid.lat*1.0e-7f,
                                grid.lon*1.0e-7f,
                                grid.height[0][0]);
            Location loc2;
            loc2.lat = grid.lat;
            loc2.lng = grid.lon;
            location_offset(loc2, 28*grid_spacing, 32*grid_spacing);
            hal.console->printf("--lat=%12.7f --lon=%12.7f %u\n",
                                loc2.lat*1.0e-7f,
                                loc2.lng*1.0e-7f,
                                grid.height[27][31]);            
        }
#endif
}

/*
  update terrain data. Check if we need to request more grids. This
  should be called at 1Hz
 */
void AP_Terrain::update(void)
{
    Location loc;
    if (!ahrs.get_position(loc)) {
        // we don't know where we are
        return;
    }
#if TERRAIN_DEBUG
    float height;
    if (height_amsl(loc, height)) {
        printf("height %.2f\n", height);
    }
#endif
}

#endif // HAVE_AP_TERRAIN
