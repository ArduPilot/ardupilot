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

#if HAVE_AP_TERRAIN

#include <assert.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>

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
    last_request_time_ms(0),
    disk_io_state(DiskIoIdle),
    fd(-1),
    timer_setup(false),
    file_lat_degrees(0),
    file_lon_degrees(0),
    io_failure(false),
    directory_created(false)
{
    AP_Param::setup_object_defaults(this, var_info);
}

/*
  return terrain height in meters above average sea level (WGS84) for
  a given position

  This is the base function that other height calculations are derived
  from. The functions below are more convenient for most uses
 */
bool AP_Terrain::height_amsl(const Location &loc, float &height)
{
    if (!enable) {
        return false;
    }

    // quick access for home altitude
    if (loc.lat == home_loc.lat &&
        loc.lng == home_loc.lng) {
        height = home_height;
        return true;
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

    // do a simple dual linear interpolation. We could do something
    // fancier, but it probably isn't worth it as long as the
    // grid_spacing is kept small enough
    float avg1 = (1.0f-info.frac_x) * h00  + info.frac_x * h10;
    float avg2 = (1.0f-info.frac_x) * h01  + info.frac_x * h11;
    float avg  = (1.0f-info.frac_y) * avg1 + info.frac_y * avg2;

    height = avg;

    if (loc.lat == ahrs.get_home().lat &&
        loc.lng == ahrs.get_home().lng) {
        // remember home altitude as a special case
        home_height = height;
        home_loc = loc;
    }

    return true;
}


/* 
   find difference between home terrain height and the terrain height
   at a given location, in meters. A positive result means the terrain
   is higher than home.

   return false is terrain at the given location or at home
   location is not available
*/
bool AP_Terrain::height_terrain_difference_home(const Location &loc, float &terrain_difference)
{
    float height_home, height_loc;
    if (!height_amsl(ahrs.get_home(), height_home)) {
        // we don't know the height of home
        return false;
    }

    if (!height_amsl(loc, height_loc)) {
        // we don't know the height of the given location
        return false;
    }

    terrain_difference = height_loc - height_home;

    return true;
}

/* 
   return estimated height above the terrain given a relative-to-home
   altitude (such as a barometric altitude) for a given location
       
   return false if terrain data is not available either at the given
   location or at the home location.  
*/
bool AP_Terrain::height_above_terrain(const Location &loc, float relative_home_altitude, float &terrain_altitude)
{
    float terrain_difference;
    if (!height_terrain_difference_home(loc, terrain_difference)) {
        return false;
    }
    terrain_altitude = relative_home_altitude - terrain_difference;
    return true;
}

/* 
   return estimated height above the terrain given a relative-to-home
   altitude (such as a barometric altitude) for a given location
       
   return false if terrain data is not available either at the given
   location or at the home location.  
*/
bool AP_Terrain::height_above_terrain(const Location &loc, float &terrain_altitude)
{
    float relative_home_altitude = loc.alt;
    if (!loc.flags.relative_alt) {
        // loc.alt has home altitude added, remove it
        relative_home_altitude -= ahrs.get_home().alt*0.01f;
    }
    return height_above_terrain(loc, relative_home_altitude, terrain_altitude);
}

/* 
   return estimated equivalent relative-to-home altitude in meters of
   a given height above the terrain for a given location.

   This function allows existing height controllers which work on
   barometric altitude (relative to home) to be used with terrain
   based target altitude, by translating the "above terrain" altitude
   into an equivalent barometric relative height.

   return false if terrain data is not available either at the given
   location or at the home location.  
*/
bool AP_Terrain::height_relative_home_equivalent(const Location &loc, float terrain_altitude, float &relative_home_altitude)
{
    float terrain_difference;
    if (!height_terrain_difference_home(loc, terrain_difference)) {
        return false;
    }
    relative_home_altitude = terrain_altitude + terrain_difference;
    return true;
}


/*
  1Hz update function. This is here to ensure progress is made on disk
  IO even if no MAVLink send_request() operations are called for a
  while.
 */
void AP_Terrain::update(void)
{
    // just schedule any needed disk IO
    schedule_disk_io();

    // try to ensure the home location is populated
    float height;
    height_amsl(ahrs.get_home(), height);
}

#endif // HAVE_AP_TERRAIN
