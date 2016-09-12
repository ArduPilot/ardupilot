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

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/GCS.h>
#include <DataFlash/DataFlash.h>
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
const AP_Param::GroupInfo AP_Terrain::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Terrain data enable
    // @Description: enable terrain data. This enables the vehicle storing a database of terrain data on the SD card. The terrain data is requested from the ground station as needed, and stored for later use on the SD card. To be useful the ground station must support TERRAIN_REQUEST messages and have access to a terrain database, such as the SRTM database.
    // @Values: 0:Disable,1:Enable
    AP_GROUPINFO_FLAGS("ENABLE", 0, AP_Terrain, enable, 1, AP_PARAM_FLAG_ENABLE),

    // @Param: SPACING
    // @DisplayName: Terrain grid spacing
    // @Description: Distance between terrain grid points in meters. This controls the horizontal resolution of the terrain data that is stored on te SD card and requested from the ground station. If your GCS is using the worldwide SRTM database then a resolution of 100 meters is appropriate. Some parts of the world may have higher resolution data available, such as 30 meter data available in the SRTM database in the USA. The grid spacing also controls how much data is kept in memory during flight. A larger grid spacing will allow for a larger amount of data in memory. A grid spacing of 100 meters results in the vehicle keeping 12 grid squares in memory with each grid square having a size of 2.7 kilometers by 3.2 kilometers. Any additional grid squares are stored on the SD once they are fetched from the GCS and will be demand loaded as needed.
    // @Units: meters
    // @Increment: 1
    AP_GROUPINFO("SPACING",   1, AP_Terrain, grid_spacing, 100),

    AP_GROUPEND
};

// constructor
AP_Terrain::AP_Terrain(AP_AHRS &_ahrs, const AP_Mission &_mission, const AP_Rally &_rally) :
    ahrs(_ahrs),
    mission(_mission),
    rally(_rally),
    disk_io_state(DiskIoIdle),
    fd(-1),
    timer_setup(false),
    file_lat_degrees(0),
    file_lon_degrees(0),
    io_failure(false),
    directory_created(false),
    home_height(0),
    have_current_loc_height(false),
    last_current_loc_height(0)
{
    AP_Param::setup_object_defaults(this, var_info);
    memset(&home_loc, 0, sizeof(home_loc));
    memset(&disk_block, 0, sizeof(disk_block));
    memset(last_request_time_ms, 0, sizeof(last_request_time_ms));
}

/*
  return terrain height in meters above average sea level (WGS84) for
  a given position

  This is the base function that other height calculations are derived
  from. The functions below are more convenient for most uses

  This function costs about 20 microseconds on Pixhawk
 */
bool AP_Terrain::height_amsl(const Location &loc, float &height, bool corrected)
{
    if (!enable || !allocate()) {
        return false;
    }

    // quick access for home altitude
    if (loc.lat == home_loc.lat &&
        loc.lng == home_loc.lng) {
        height = home_height;
        // apply correction which assumes home altitude is at terrain altitude
        if (corrected) {
            height += (ahrs.get_home().alt * 0.01f) - home_height;
        }
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

    // apply correction which assumes home altitude is at terrain altitude
    if (corrected) {
        height += (ahrs.get_home().alt * 0.01f) - home_height;
    }

    return true;
}


/* 
   find difference between home terrain height and the terrain
   height at the current location in meters. A positive result
   means the terrain is higher than home.

   return false is terrain at the current location or at home
   location is not available

   If extrapolate is true then allow return of an extrapolated
   terrain altitude based on the last available data
*/
bool AP_Terrain::height_terrain_difference_home(float &terrain_difference, bool extrapolate)
{
    float height_home, height_loc;
    if (!height_amsl(ahrs.get_home(), height_home, false)) {
        // we don't know the height of home
        return false;
    }

    Location loc;
    if (!ahrs.get_position(loc)) {
        // we don't know where we are
        return false;
    }

    if (!height_amsl(loc, height_loc, false)) {
        if (!extrapolate || !have_current_loc_height) {
            // we don't know the height of the given location
            return false;
        }
        // we don't have data at the current location, but the caller
        // has asked for extrapolation, so use the last available
        // terrain height. This can be used to fill in while new data
        // is fetched. It should be very rarely used
        height_loc = last_current_loc_height;
    }

    terrain_difference = height_loc - height_home;

    return true;
}

/* 
   return current height above terrain at current AHRS
   position. 
   
   If extrapolate is true then extrapolate from most recently
   available terrain data is terrain data is not available for the
   current location.
   
   Return true if height is available, otherwise false.
*/
bool AP_Terrain::height_above_terrain(float &terrain_altitude, bool extrapolate)
{
    float terrain_difference;
    if (!height_terrain_difference_home(terrain_difference, extrapolate)) {
        return false;
    }
    Location loc;
    if (!ahrs.get_position(loc)) {
        // we don't know where we are
        return false;
    }
    float relative_home_altitude = loc.alt*0.01f;
    if (!loc.flags.relative_alt) {
        // loc.alt has home altitude added, remove it
        relative_home_altitude -= ahrs.get_home().alt*0.01f;
    }
    terrain_altitude = relative_home_altitude - terrain_difference;
    return true;
}

/* 
   return estimated equivalent relative-to-home altitude in meters
   of a given height above the terrain at the current location
       
   This function allows existing height controllers which work on
   barometric altitude (relative to home) to be used with terrain
   based target altitude, by translating the "above terrain" altitude
   into an equivalent barometric relative height.
   
   return false if terrain data is not available either at the given
   location or at the home location.  
   
   If extrapolate is true then allow return of an extrapolated
   terrain altitude based on the last available data
*/
bool AP_Terrain::height_relative_home_equivalent(float terrain_altitude, 
                                                 float &relative_home_altitude,
                                                 bool extrapolate)
{
    float terrain_difference;
    if (!height_terrain_difference_home(terrain_difference, extrapolate)) {
        return false;
    }
    relative_home_altitude = terrain_altitude + terrain_difference;
    return true;
}


/*
  calculate lookahead rise in terrain. This returns extra altitude
  needed to clear upcoming terrain in meters
*/
float AP_Terrain::lookahead(float bearing, float distance, float climb_ratio)
{
    if (!enable || !allocate() || grid_spacing <= 0) {
        return 0;
    }

    Location loc;
    if (!ahrs.get_position(loc)) {
        // we don't know where we are
        return 0;
    }
    float base_height;
    if (!height_amsl(loc, base_height, false)) {
        // we don't know our current terrain height
        return 0;
    }

    float climb = 0;
    float lookahead_estimate = 0;

    // check for terrain at grid spacing intervals
    while (distance > 0) {
        location_update(loc, bearing, grid_spacing);
        climb += climb_ratio * grid_spacing;
        distance -= grid_spacing;
        float height;
        if (height_amsl(loc, height, false)) {
            float rise = (height - base_height) - climb;
            if (rise > lookahead_estimate) {
                lookahead_estimate = rise;
            }
        }
    }

    return lookahead_estimate;
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
    height_amsl(ahrs.get_home(), height, false);

    // update the cached current location height
    Location loc;
    bool pos_valid = ahrs.get_position(loc);
    bool terrain_valid = height_amsl(loc, height, false);
    if (pos_valid && terrain_valid) {
        last_current_loc_height = height;
        have_current_loc_height = true;
    }

    // check for pending mission data
    update_mission_data();

    // check for pending rally data
    update_rally_data();

    // update capabilities and status
    if (enable) {
        hal.util->set_capabilities(MAV_PROTOCOL_CAPABILITY_TERRAIN);
        if (!pos_valid) {
            // we don't know where we are
            system_status = TerrainStatusUnhealthy;
        } else if (!terrain_valid) {
            // we don't have terrain data at current location
            system_status = TerrainStatusUnhealthy;
        } else {
            system_status = TerrainStatusOK;
        }
    } else {
        hal.util->clear_capabilities(MAV_PROTOCOL_CAPABILITY_TERRAIN);
        system_status = TerrainStatusDisabled;
    }

}

/*
  log terrain data to dataflash log
 */
void AP_Terrain::log_terrain_data(DataFlash_Class &dataflash)
{
    if (!enable) {
        return;
    }
    Location loc;
    if (!ahrs.get_position(loc)) {
        // we don't know where we are
        return;
    }
    float terrain_height = 0;
    float current_height = 0;
    uint16_t pending, loaded;

    height_amsl(loc, terrain_height, false);
    height_above_terrain(current_height, true);
    get_statistics(pending, loaded);

    struct log_TERRAIN pkt = {
        LOG_PACKET_HEADER_INIT(LOG_TERRAIN_MSG),
        time_us        : AP_HAL::micros64(),
        status         : (uint8_t)status(),
        lat            : loc.lat,
        lng            : loc.lng,
        spacing        : (uint16_t)grid_spacing,
        terrain_height : terrain_height,
        current_height : current_height,
        pending        : pending,
        loaded         : loaded
    };
    dataflash.WriteBlock(&pkt, sizeof(pkt));
}

/*
  allocate terrain cache. Making this dynamically allocated allows
  memory to be saved when terrain functionality is disabled
 */
bool AP_Terrain::allocate(void)
{
    if (enable == 0) {
        return false;
    }
    if (cache != nullptr) {
        return true;
    }
    cache = (struct grid_cache *)calloc(TERRAIN_GRID_BLOCK_CACHE_SIZE, sizeof(cache[0]));
    if (cache == nullptr) {
        enable.set(0);
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "Terrain: Allocation failed");
        return false;
    }
    cache_size = TERRAIN_GRID_BLOCK_CACHE_SIZE;
    return true;
}

#endif // AP_TERRAIN_AVAILABLE
