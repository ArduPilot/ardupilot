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

#if AP_TERRAIN_AVAILABLE

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
    // @Description: enable terrain following. This enables the vehicle storing a database of terrain data on the SD card. The terrain data is requested from the ground station as needed, and stored for later use on the SD card. To be useful the ground station must support TERRAIN_REQUEST messages and have access to a terrain database, such as the SRTM database.
    // @Values: 0:Disable,1:Enable
    AP_GROUPINFO("ENABLE",    0, AP_Terrain, enable, 0),

    // @Param: SPACING
    // @DisplayName: Terrain grid spacing
    // @Description: Distance between terrain grid points in meters. This controls the horizontal resolution of the terrain data that is stored on te SD card and requested from the ground station. If your GCS is using the worldwide SRTM database then a resolution of 100 meters is appropriate. Some parts of the world may have higher resolution data available, such as 30 meter data available in the SRTM database in the USA. The grid spacing also controls how much data is kept in memory during flight. A larger grid spacing will allow for a larger amount of data in memory. A grid spacing of 100 meters results in the vehicle keeping 12 grid squares in memory with each grid square having a size of 2.7 kilometers by 3.2 kilometers. Any additional grid squares are stored on the SD once they are fetched from the GCS and will be demand loaded as needed.
    // @Units: meters
    // @Increment: 1
    AP_GROUPINFO("SPACING",   1, AP_Terrain, grid_spacing, 100),

    AP_GROUPEND
};

// constructor
AP_Terrain::AP_Terrain(AP_AHRS &_ahrs) :
    ahrs(_ahrs),
    disk_io_state(DiskIoIdle),
    last_request_time_ms(0),
    fd(-1),
    timer_setup(false),
    file_lat_degrees(0),
    file_lon_degrees(0),
    io_failure(false),
    directory_created(false),
    home_height(0)
{
    AP_Param::setup_object_defaults(this, var_info);
    memset(&home_loc, 0, sizeof(home_loc));
    memset(&disk_block, 0, sizeof(disk_block));
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
    const struct grid_block &grid = find_grid_cache(info).grid;

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
   alternative interface to height_above_terrain where
   relative_altitude is taken from loc.alt (in centimeters)
*/
bool AP_Terrain::height_above_terrain(const Location &loc, float &terrain_altitude)
{
    float relative_home_altitude = loc.alt*0.01f;
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
   convert a Location altitude to a relative-to-home altitude in meters
   This obeys the relative_alt and terrain_alt flags in Location.flags
*/
bool AP_Terrain::location_to_relative_home(const Location &loc, float &relative_altitude)
{
    if (!loc.flags.terrain_alt) {
        // its not a terrain alt
        relative_altitude = loc.alt*0.01f;
        if (!loc.flags.relative_alt) {
            relative_altitude -= ahrs.get_home().alt*0.01f;
        }
        return true;
    }

    if (!height_relative_home_equivalent(loc, loc.alt*0.01f, relative_altitude)) {
        return false;
    }

    // if terrain_alt is set and relative_alt is not set then Location
    // is still offset by home alt
    if (!loc.flags.relative_alt) {
        relative_altitude -= ahrs.get_home().alt*0.01f;
    }
    return true;
}


/*
  1hz update function. This is here to ensure progress is made on disk
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

/*
  return status enum for health reporting
*/
enum AP_Terrain::TerrainStatus AP_Terrain::status(void)
{
    if (!enable) {
        return TerrainStatusDisabled;
    }
    Location loc;
    if (!ahrs.get_position(loc)) {
        // we don't know where we are
        return TerrainStatusUnhealthy;
    }
    float height;
    if (!height_amsl(loc, height)) {
        // we don't have terrain data at current location
        return TerrainStatusUnhealthy;        
    }
    return TerrainStatusOK; 
}

#endif // AP_TERRAIN_AVAILABLE
