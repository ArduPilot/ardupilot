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

#include "AP_Terrain.h"

#if AP_TERRAIN_AVAILABLE

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_Filesystem/AP_Filesystem.h>

extern const AP_HAL::HAL& hal;

AP_Terrain *AP_Terrain::singleton;

#if APM_BUILD_TYPE(APM_BUILD_ArduSub)
#define TERRAIN_ENABLE_DEFAULT 0
#else
#define TERRAIN_ENABLE_DEFAULT 1
#endif

// table of user settable parameters
const AP_Param::GroupInfo AP_Terrain::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Terrain data enable
    // @Description: enable terrain data. This enables the vehicle storing a database of terrain data on the SD card. The terrain data is requested from the ground station as needed, and stored for later use on the SD card. To be useful the ground station must support TERRAIN_REQUEST messages and have access to a terrain database, such as the SRTM database.
    // @Values: 0:Disable,1:Enable
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 0, AP_Terrain, enable, TERRAIN_ENABLE_DEFAULT, AP_PARAM_FLAG_ENABLE),

    // @Param: SPACING
    // @DisplayName: Terrain grid spacing
    // @Description: Distance between terrain grid points in meters. This controls the horizontal resolution of the terrain data that is stored on te SD card and requested from the ground station. If your GCS is using the ArduPilot SRTM database like Mission Planner or MAVProxy, then a resolution of 100 meters is appropriate. Grid spacings lower than 100 meters waste SD card space if the GCS cannot provide that resolution. The grid spacing also controls how much data is kept in memory during flight. A larger grid spacing will allow for a larger amount of data in memory. A grid spacing of 100 meters results in the vehicle keeping 12 grid squares in memory with each grid square having a size of 2.7 kilometers by 3.2 kilometers. Any additional grid squares are stored on the SD once they are fetched from the GCS and will be loaded as needed.
    // @Units: m
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("SPACING",   1, AP_Terrain, grid_spacing, 100),

    // @Param: OPTIONS
    // @DisplayName: Terrain options
    // @Description: Options to change behaviour of terrain system
    // @Bitmask: 0:Disable Download
    // @User: Advanced
    AP_GROUPINFO("OPTIONS",   2, AP_Terrain, options, 0),

    // @Param: MARGIN
    // @DisplayName: Acceptance margin
    // @Description: Margin in centi-meters to accept terrain data from the GCS. This can be used to allow older terrain data generated with less accurate latitude/longitude scaling to be used
    // @Units: m
    // @Range: 0.05 50000
    // @User: Advanced
    AP_GROUPINFO("MARGIN",   3, AP_Terrain, margin, 0.05),

    // @Param: OFS_MAX
    // @DisplayName: Terrain reference offset maximum
    // @Description: The maximum adjustment of terrain altitude based on the assumption that the vehicle is on the ground when it is armed. When the vehicle is armed the location of the vehicle is recorded, and when terrain data is available for that location a height adjustment for terrain data is calculated that aligns the terrain height at that location with the altitude recorded at arming. This height adjustment is applied to all terrain data. This parameter clamps the amount of adjustment. A value of zero disables the use of terrain height adjustment.
    // @Units: m
    // @Range: 0 50
    // @User: Advanced
    AP_GROUPINFO("OFS_MAX",  4, AP_Terrain, offset_max, 30),
    
    AP_GROUPEND
};

// constructor
AP_Terrain::AP_Terrain() :
    disk_io_state(DiskIoIdle),
    fd(-1)
{
    AP_Param::setup_object_defaults(this, var_info);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (singleton != nullptr) {
        AP_HAL::panic("Terrain must be singleton");
    }
#endif
    singleton = this;
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
    if (!allocate()) {
        return false;
    }

    const AP_AHRS &ahrs = AP::ahrs();

    // quick access for home altitude
    if (loc.lat == home_loc.lat &&
        loc.lng == home_loc.lng) {
        height = home_height;
        if (corrected && have_reference_offset) {
            height += reference_offset;
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
        have_home_height = true;
    }

    if (corrected && have_reference_offset) {
        height += reference_offset;
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
    const AP_AHRS &ahrs = AP::ahrs();

    float height_home, height_loc;
    if (!height_amsl(ahrs.get_home(), height_home)) {
        // we don't know the height of home
        return false;
    }

    Location loc;
    if (!ahrs.get_location(loc)) {
        // we don't know where we are
        return false;
    }

    if (!height_amsl(loc, height_loc)) {
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

    float relative_home_altitude;
    AP::ahrs().get_relative_position_D_home(relative_home_altitude);
    relative_home_altitude = -relative_home_altitude;

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
    if (!allocate() || grid_spacing <= 0) {
        return 0;
    }

    Location loc;
    if (!AP::ahrs().get_location(loc)) {
        // we don't know where we are
        return 0;
    }
    float base_height;
    if (!height_amsl(loc, base_height)) {
        // we don't know our current terrain height
        return 0;
    }

    float climb = 0;
    float lookahead_estimate = 0;

    // check for terrain at grid spacing intervals
    while (distance > 0) {
        loc.offset_bearing(bearing, grid_spacing);
        climb += climb_ratio * grid_spacing;
        distance -= grid_spacing;
        float height;
        if (height_amsl(loc, height)) {
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
    if (!enable) { return; }
    // just schedule any needed disk IO
    schedule_disk_io();

    const AP_AHRS &ahrs = AP::ahrs();

    // try to ensure the home location is populated
    float height;
    height_amsl(ahrs.get_home(), height);

    // update the cached current location height
    Location loc;
    bool pos_valid = ahrs.get_location(loc);
    bool terrain_valid = pos_valid && height_amsl(loc, height);
    if (pos_valid && terrain_valid) {
        last_current_loc_height = height;
        have_current_loc_height = true;
    }

    // check for pending mission data
    update_mission_data();

#if HAL_RALLY_ENABLED
    // check for pending rally data
    update_rally_data();
#endif

    // update tiles surrounding our current location:
    if (pos_valid) {
        have_surrounding_tiles = update_surrounding_tiles(loc);
    } else {
        have_surrounding_tiles = false;
    }

    // update capabilities and status
    if (allocate()) {
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
        system_status = TerrainStatusDisabled;
    }
}

bool AP_Terrain::update_surrounding_tiles(const Location &loc)
{
    // also request a larger set of up to 9 grids
    bool ret = true;
    for (int8_t x=-1; x<=1; x++) {
        for (int8_t y=-1; y<=1; y++) {
            Location loc2 = loc;
            loc2.offset(x*TERRAIN_GRID_BLOCK_SIZE_X*0.7f*grid_spacing,
                        y*TERRAIN_GRID_BLOCK_SIZE_Y*0.7f*grid_spacing);
            float height;
            if (!height_amsl(loc2, height)) {
                ret = false;
            }
        }
    }
    return ret;
}

bool AP_Terrain::pre_arm_checks(char *failure_msg, uint8_t failure_msg_len) const
{
    // check no outstanding requests for data:
    uint16_t terr_pending, terr_loaded;
    get_statistics(terr_pending, terr_loaded);
    if (terr_pending != 0 ||
        !have_current_loc_height ||
        !have_home_height ||
        next_mission_index != 0 ||
        next_rally_index != 0) {
        hal.util->snprintf(failure_msg, failure_msg_len, "waiting for terrain data");
        return false;
    }

    return true;
}

void AP_Terrain::log_terrain_data()
{
    if (!allocate()) {
        return;
    }
    Location loc;
    if (!AP::ahrs().get_location(loc)) {
        // we don't know where we are
        return;
    }
    float terrain_height = 0;
    float current_height = 0;
    uint16_t pending, loaded;

    height_amsl(loc, terrain_height);
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
        loaded         : loaded,
        reference_offset : have_reference_offset?reference_offset:0,
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

/*
  allocate terrain cache. Making this dynamically allocated allows
  memory to be saved when terrain functionality is disabled
 */
bool AP_Terrain::allocate(void)
{
    if (enable == 0 || memory_alloc_failed) {
        return false;
    }
    if (cache != nullptr) {
        return true;
    }
    cache = (struct grid_cache *)calloc(TERRAIN_GRID_BLOCK_CACHE_SIZE, sizeof(cache[0]));
    if (cache == nullptr) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Terrain: Allocation failed");
        memory_alloc_failed = true;
        return false;
    }
    cache_size = TERRAIN_GRID_BLOCK_CACHE_SIZE;
    return true;
}

/*
  setup a reference location for terrain adjustment. This should
  be called when the vehicle is definately on the ground
*/
void AP_Terrain::set_reference_location(void)
{
    const auto &ahrs = AP::ahrs();

    // check we have absolute position
    nav_filter_status status;
    if (!ahrs.get_filter_status(status) ||
        !status.flags.vert_pos ||
        !status.flags.horiz_pos_abs ||
        !status.flags.attitude) {
        return;
    }

    // check we have a small 3D velocity
    Vector3f vel;
    if (!ahrs.get_velocity_NED(vel) ||
        vel.length() > 3) {
        return;
    }

    have_reference_offset = false;
    have_reference_loc = ahrs.get_location(reference_loc);

    update_reference_offset();
}

/*
  get the offset between terrain height and reference alt at the
  reference location
 */
void AP_Terrain::update_reference_offset(void)
{
    // TERR_OFS_MAX of zero means no adjustment
    if (!is_positive(offset_max)) {
        have_reference_offset = false;
        return;
    }

    // allow for change to TERRAIN_OFS_MAX while flying
    if (have_reference_offset) {
        reference_offset = constrain_float(reference_offset, -offset_max, offset_max);
        return;
    }

    if (!have_reference_loc) {
        // no reference available yet
        return;
    }

    // calculate adjustment
    float height;
    if (!height_amsl(reference_loc, height)) {
        return;
    }
    int32_t alt_cm;
    if (!reference_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, alt_cm)) {
        return;
    }
    float adjustment = alt_cm*0.01 - height;
    reference_offset = constrain_float(adjustment, -offset_max, offset_max);
    if (fabsf(adjustment) > offset_max.get()+0.5) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Terrain: clamping offset %.0f to %.0f",
                      adjustment, reference_offset);
    }
    have_reference_offset = true;
}


namespace AP {

AP_Terrain *terrain()
{
    return AP_Terrain::get_singleton();
}

};

#endif // AP_TERRAIN_AVAILABLE
