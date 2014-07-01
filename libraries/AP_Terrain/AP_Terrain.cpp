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
#include "AP_Terrain.h"

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
    grid_cache(NULL),
    last_request_time_ms(0)
{
    AP_Param::setup_object_defaults(this, var_info);
}

/*
  given a location, calculate the 45x45 grid SW corner, plus the
  grid index and grid square fraction
*/
void AP_Terrain::calculate_grid_info(const Location &loc, struct grid_info &info) const
{
    // grids start on integer degrees. This makes storing terrain data
    // on the SD card a bit easier
    info.lat_degrees = loc.lat / 10*1000*1000UL;
    info.lon_degrees = loc.lng / 10*1000*1000UL;

    // create reference position. Longitude scaling is taken from this point
    Location ref;
    ref.lat = info.lat_degrees;
    ref.lng = info.lon_degrees;

    // find offset from reference
    Vector2f offset = location_diff(ref, loc);

    // work out how many 45x45 grid squares we are in. x is north, y is east
    info.idx_x = ((uint16_t)(offset.x / grid_spacing))/TERRAIN_GRID_BLOCK_SIZE;
    info.idx_y = ((uint16_t)(offset.y / grid_spacing))/TERRAIN_GRID_BLOCK_SIZE;

    // work out fractional (0 to 1) position within grid square.
    info.frac_x = (offset.x - (info.idx_x * (float)TERRAIN_GRID_BLOCK_SIZE * grid_spacing)) / grid_spacing;
    info.frac_y = (offset.y - (info.idx_y * (float)TERRAIN_GRID_BLOCK_SIZE * grid_spacing)) / grid_spacing;

    // calculate lat/lon of SW corner of 45x45 grid_block
    location_offset(ref, info.idx_x*grid_spacing, info.idx_y*grid_spacing);
    info.grid_lat = ref.lat;
    info.grid_lon = ref.lng;

    // calculate bit number in grid_block bitmap
    info.bitnum = (info.idx_y/TERRAIN_GRID_MAVLINK_SIZE)*TERRAIN_GRID_BLOCK_MUL + info.idx_x/TERRAIN_GRID_MAVLINK_SIZE;
}


/*
  given a location and offset, calculate the 45x45 grid SW corner, plus
  the grid index and grid square fraction 
*/
void AP_Terrain::calculate_grid_info(const Location &loc, struct grid_info &info, int16_t ofs_north, int16_t ofs_east) const
{
    Location loc2 = loc;
    location_offset(loc2, ofs_north, ofs_east);
    calculate_grid_info(loc2, info);
}

/*
  find a grid structure given a grid_info
 */
AP_Terrain::grid_block &AP_Terrain::find_grid(const struct grid_info &info) const
{
    uint16_t oldest_i = 0;

    // see if we have that grid
    for (uint16_t i=0; i<TERRAIN_GRID_BLOCK_CACHE_SIZE; i++) {
        if (grid_cache[i].lat == info.grid_lat && 
            grid_cache[i].lon == info.grid_lon) {
            grid_cache[i].last_access_ms = hal.scheduler->millis();
            return &grid_cache[i];
        }
        if (grid_cache[i].last_access_ms < grid_cache[oldest_i].last_access_ms) {
            oldest_i = i;
        }
    }

    // Not found. Use the oldest grid and make it this grid,
    // unpopulated
    struct grid_block &grid = grid_cache[oldest_i];
    memset(&grid, 0, sizeof(grid));

    grid.lat = info.grid_lat;
    grid.lon = info.grid_lon;
    grid.spacing = grid_spacing;

    return grid;
}

/*
  return terrain height in meters above average sea level (WGS84) for
  a grid_info, returning the height for the SW corner of the grid square
 */
bool AP_Terrain::height_sw_corner(const struct grid_info &info, int16_t &height)
{
    struct grid_block &grid = find_grid(info);
    if (grid.bitmask[info.bitnum/8] & (1U<<(info.bitnum%8))) {
        // we have the height
        height = grid.height[info.idx_x][info.idx_y];
        return true;
    }
    return false;
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

    struct grid_info info00, info01, info10, info11;

    // we push the spacing up a bit to cope with changes in longitude
    // scaling between grids
    uint16_t spacing2 = spacing*1.2f;
    calculate_grid_info(loc, info00);
    calculate_grid_info(loc, info01, 0, spacing2);
    calculate_grid_info(loc, info10, spacing2, 0);
    calculate_grid_info(loc, info11, spacing2, spacing2);

    // hXY are the heights of the 4 surrounding grid points
    int16_t h00, h01, h10, h11;

    if (!height_sw_corner(info00, h00) ||
        !height_sw_corner(info01, h01) ||
        !height_sw_corner(info10, h10) ||
        !height_sw_corner(info11, h11)) {
        // we don't have the data on all sided
        return false;
    }

    // TODO: cope with crossing degree boundaries

    float avg1 = (1.0f-info00.frac_x) * h00 + info00.frac_x * h10;
    float avg2 = (1.0f-info00.frac_x) * h01 + info00.frac_x * h11;
    float avg = (1.0f-info00.frac_y) * avg1 + info00.frac_y * avg2;

    height = avg;
    return true;
}


/*
  update terrain data. Check if we need to request more grids. This
  should be called at 1Hz
 */
void AP_Terrain::update(void)
{
    if (enable == 0) {
        // not enabled
        return;
    }
    Location loc;
    if (!ahrs.get_position(loc)) {
        // we don't know where we are
        return;
    }

    // find any missing 5x5 blocks in the current grid
    struct grid_info info;
    calculate_grid_info(loc, info);

    if (request_missing(info)) {
        return;
    }
}

void AP_Terrain::request_missing(struct grid_info &info)
{
    
}
