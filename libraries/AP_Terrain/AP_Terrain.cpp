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

    AP_GROUPEND
};

// constructor
AP_Terrain::AP_Terrain(AP_AHRS &_ahrs) :
    ahrs(_ahrs),
    last_grid_spacing(0),
    grids_allocated(0),
    grids(NULL),
    last_request_time_ms(0)
{
    AP_Param::setup_object_defaults(this, var_info);
}

/*
  allocate terrain object grid memory if enabled.
 */
void AP_Terrain::allocate(void)
{
    if (enable == 0) {
        return;
    }
    // constrain grid size to avoid 16 bit overflow
    if (grid_width > 150) {
        grid_width.set(150);
    }
    if (grid_width < 0) {
        return;
    }
    uint16_t grids_needed = sq((grid_width+4) / 5);
    uint16_t memory_needed = grids_needed * sizeof(struct grid);
    if (hal.util->available_memory() < memory_needed+512) {
        // refuse to allocate last bit of memory, we need some for
        // stack
        return;
    }
    if (grids != NULL && grids_needed == grids_allocated) {
        // already allocated
        return;
    }
    if (grids != NULL) {
        free(grids);
    }
    grids = (struct grid *)calloc(grids_needed, sizeof(struct grid));
    if (grids == NULL) {
        // not enough memory
        return;
    }
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
    // re-allocate if need be
    allocate();
}

/*
  given a location, calculate the 5x5 grid NW corner, plus the
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

    // work out how many 5x5 grid squares we are in. x is north, y is east
    info.idx_x = ((uint16_t)(offset.x / grid_spacing))/5;
    info.idx_y = ((uint16_t)(offset.y / grid_spacing))/5;

    // work out fractional (0 to 1) position within grid square.
    info.frac_x = (offset.x - (info.idx_x * 5.0f * grid_spacing)) / grid_spacing;
    info.frac_y = (offset.y - (info.idx_y * 5.0f * grid_spacing)) / grid_spacing;

    // calculate lat/lon of SW corner of 5x5 grid
    location_offset(ref, info.idx_x*grid_spacing, info.idx_y*grid_spacing);
    info.grid_lat = ref.lat;
    info.grid_lon = ref.lng;
}

/*
  find a grid structure given a location and offset in meters
 */
const AP_Terrain::grid *AP_Terrain::find_grid(const Location &loc, uint16_t ofs_north, uint16_t ofs_east) const
{
    struct grid_info info;
    Location loc2 = loc;
    location_offset(loc2, ofs_north, ofs_east);
    calculate_grid_info(loc2, info);

    // see if we have that grid
    for (uint16_t i=0; i<grids_allocated; i++) {
        if (grids[i].valid && 
            grids[i].lat == info.grid_lat && 
            grids[i].lon == info.grid_lon) {
            return &grids[i];
        }
    }

    // not found
    return NULL;
}


/*
  return terrain height in meters above average sea level (WGS84) for
  a given position
 */
bool AP_Terrain::height_amsl(const Location &loc, float &height)
{
    if (!enable || grids == NULL) {
        return false;
    }

    struct grid_info info;
    calculate_grid_info(loc, info);

    // see if we have that grid
    uint16_t i;
    for (i=0; i<grids_allocated; i++) {
        if (grids[i].valid && 
            grids[i].lat == info.grid_lat && 
            grids[i].lon == info.grid_lon) {
            // found it, interpolate within the grid
            break;
        }
    }
    if (i == grids_allocated) {
        // not found
        return false;
    }

    // hXY are the heights of the 4 surrounding grid points
    int16_t h00, h01, h10, h11;

    // we can get h00 now
    h00 = grids[i].height[info.idx_x][info.idx_y];

    // when finding neighbouring grids ask for grid_spacing*2
    // to cope with rounding resulting in gaps between grids
    const uint16_t grid_sep = grid_spacing*2;

    // maximum index within a grid
    const uint8_t max_idx = 5-1;

    // do we cross into another grid?
    if (info.idx_x < max_idx && info.idx_y < max_idx) {
        h01 = grids[i].height[info.idx_x][info.idx_y+1];
        h10 = grids[i].height[info.idx_x+1][info.idx_y];
        h11 = grids[i].height[info.idx_x+1][info.idx_y+1];
    } else if (info.idx_x == max_idx && info.idx_y < max_idx) {
        // we need the grid above this one
        // note that we use 
        const grid *grid2 = find_grid(loc, grid_sep, 0);
        if (grid2 == NULL) {
            return false;
        }
        h01 = grids[i].height[info.idx_x][info.idx_y+1];
        h10 = grid2->height[0][info.idx_y];
        h11 = grid2->height[0][info.idx_y+1];                
    } else if (info.idx_x < max_idx && info.idx_y == max_idx) {
        // we need the grid to the right of this one
        const grid *grid2 = find_grid(loc, 0, grid_sep);
        if (grid2 == NULL) {
            return false;
        }
        h01 = grid2->height[info.idx_x][0];
        h10 = grids[i].height[info.idx_x+1][info.idx_y];
        h11 = grid2->height[info.idx_x+1][0];
    } else {
        // we need to find 3 more grids, above, right and above-right
        const grid *grid_x  = find_grid(loc, grid_sep, 0);
        const grid *grid_y  = find_grid(loc, 0, grid_sep);
        const grid *grid_xy = find_grid(loc, grid_sep, grid_sep);
        if (grid_x == NULL || grid_y == NULL || grid_xy == NULL) {
            return false;
        }
        h01 = grid_y->height[info.idx_x][0];
        h10 = grid_x->height[0][info.idx_y];
        h11 = grid_xy->height[0][0];
    }
    
    float avg1 = (1.0f-info.frac_x) * h00 + info.frac_x * h10;
    float avg2 = (1.0f-info.frac_x) * h01 + info.frac_x * h11;
    float avg = (1.0f-info.frac_y) * avg1 + info.frac_y * avg2;

    height = avg;
    return true;
}
