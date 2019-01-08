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
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <DataFlash/DataFlash.h>

#if (HAL_OS_POSIX_IO || HAL_OS_FATFS_IO) && defined(HAL_BOARD_TERRAIN_DIRECTORY)
#define AP_TERRAIN_AVAILABLE 1
#else
#define AP_TERRAIN_AVAILABLE 0
#endif

#if AP_TERRAIN_AVAILABLE

#include <AP_Param/AP_Param.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Mission/AP_Mission.h>

#define TERRAIN_DEBUG 0


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

#if TERRAIN_DEBUG
#define ASSERT_RANGE(v,minv,maxv) assert((v)<=(maxv)&&(v)>=(minv))
#else
#define ASSERT_RANGE(v,minv,maxv)
#endif


/*
  Data conventions in this library:

  array[x][y]: x is increasing north, y is increasing east
  array[x]:    low order bits increase east first
  bitmap:      low order bits increase east first
  file:        first entries increase east, then north
 */

class AP_Terrain {
public:
    AP_Terrain(const AP_Mission &_mission);

    /* Do not allow copies */
    AP_Terrain(const AP_Terrain &other) = delete;
    AP_Terrain &operator=(const AP_Terrain&) = delete;

    enum TerrainStatus {
        TerrainStatusDisabled  = 0, // not enabled
        TerrainStatusUnhealthy = 1, // no terrain data for current location
        TerrainStatusOK        = 2  // terrain data available
    };

    static const struct AP_Param::GroupInfo var_info[];

    // update terrain state. Should be called at 1Hz or more
    void update(void);

    // return status enum for health reporting
    enum TerrainStatus status(void) const { return system_status; }

    // send any pending terrain request message
    void send_request(mavlink_channel_t chan);

    // handle terrain data and reports from GCS
    void send_terrain_report(mavlink_channel_t chan, const Location &loc, bool extrapolate);
    void handle_data(mavlink_channel_t chan, mavlink_message_t *msg);
    void handle_terrain_check(mavlink_channel_t chan, mavlink_message_t *msg);
    void handle_terrain_data(mavlink_message_t *msg);

    /*
      find the terrain height in meters above sea level for a location

      return false if not available

      if corrected is true then terrain alt is adjusted so that
      the terrain altitude matches the home altitude at the home location
      (i.e. we assume home is at the terrain altitude)
     */
    bool height_amsl(const Location &loc, float &height, bool corrected);

    /* 
       find difference between home terrain height and the terrain
       height at the current location in meters. A positive result
       means the terrain is higher than home.

       return false is terrain at the current location or at home
       location is not available

       If extrapolate is true then allow return of an extrapolated
       terrain altitude based on the last available data       
    */
    bool height_terrain_difference_home(float &terrain_difference, 
                                        bool extrapolate = false);

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
    bool height_relative_home_equivalent(float terrain_altitude, 
                                         float &relative_altitude, 
                                         bool extrapolate = false);

    /* 
       return current height above terrain at current AHRS
       position. 

       If extrapolate is true then extrapolate from most recently
       available terrain data is terrain data is not available for the
       current location.

       Return true if height is available, otherwise false.
    */
    bool height_above_terrain(float &terrain_altitude, bool extrapolate = false);

    /*
      calculate lookahead rise in terrain. This returns extra altitude
      needed to clear upcoming terrain in meters
     */
    float lookahead(float bearing, float distance, float climb_ratio);

    /*
      log terrain status to DataFlash
     */
    void log_terrain_data(DataFlash_Class &dataflash);

    /*
      get some statistics for TERRAIN_REPORT
     */
    void get_statistics(uint16_t &pending, uint16_t &loaded);

private:
    // allocate the terrain subsystem data
    bool allocate(void);

    /*
      a grid block is a structure in a local file containing height
      information. Each grid block is 2048 in size, to keep file IO to
      block oriented SD cards efficient
     */
    struct PACKED grid_block {
        // bitmap of 4x4 grids filled in from GCS (56 bits are used)
        uint64_t bitmap;

        // south west corner of block in degrees*10^7
        int32_t lat;
        int32_t lon;

        // crc of whole block, taken with crc=0
        uint16_t crc;

        // format version number
        uint16_t version;

        // grid spacing in meters
        uint16_t spacing;

        // heights in meters over a 32*28 grid
        int16_t height[TERRAIN_GRID_BLOCK_SIZE_X][TERRAIN_GRID_BLOCK_SIZE_Y];

        // indices info 32x28 grids for this degree reference
        uint16_t grid_idx_x;
        uint16_t grid_idx_y;

        // rounded latitude/longitude in degrees. 
        int16_t lon_degrees;
        int8_t lat_degrees;
    };

    /*
      grid_block for disk IO, aligned on 2048 byte boundaries
     */
    union grid_io_block {
        struct grid_block block;
        uint8_t buffer[2048];
    };

    enum GridCacheState {
        GRID_CACHE_INVALID=0,    // when first initialised
        GRID_CACHE_DISKWAIT=1,   // when waiting for disk read
        GRID_CACHE_VALID=2,      // when at least partially valid
        GRID_CACHE_DIRTY=3       // when updates have been made, and
                                 // disk write needed
    };

    /*
      a grid_block plus some meta data used for requesting new blocks
     */
    struct grid_cache {
        struct grid_block grid;

        volatile enum GridCacheState state;

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

        // indices info 32x28 grids for this degree reference
        uint16_t grid_idx_x;
        uint16_t grid_idx_y;

        // indexes into 32x28 grid
        uint8_t idx_x; // north (0..27)
        uint8_t idx_y; // east  (0..31)

        // fraction within the grid square
        float frac_x; // north (0..1)
        float frac_y; // east  (0..1)

        // file offset of this grid
        uint32_t file_offset;
    };

    // given a location, fill a grid_info structure
    void calculate_grid_info(const Location &loc, struct grid_info &info) const;

    /*
      find a grid structure given a grid_info
    */
    struct grid_cache &find_grid_cache(const struct grid_info &info);

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
    bool request_missing(mavlink_channel_t chan, struct grid_cache &gcache);
    bool request_missing(mavlink_channel_t chan, const struct grid_info &info);

    /*
      look for blocks that need to be read/written to disk
     */
    void schedule_disk_io(void);

    /*
      get some statistics for TERRAIN_REPORT
     */
    uint8_t bitcount64(uint64_t b);

    /*
      disk IO functions
     */
    int16_t find_io_idx(enum GridCacheState state);
    uint16_t get_block_crc(struct grid_block &block);
    void check_disk_read(void);
    void check_disk_write(void);
    void io_timer(void);
    void open_file(void);
    void seek_offset(void);
    void write_block(void);
    void read_block(void);

    /*
      check for missing mission terrain data
     */
    void update_mission_data(void);

    /*
      check for missing rally data
     */
    void update_rally_data(void);


    // parameters
    AP_Int8  enable;
    AP_Int16 grid_spacing; // meters between grid points

    // reference to AP_Mission, so we can ask preload terrain data for 
    // all waypoints
    const AP_Mission &mission;

    // cache of grids in memory, LRU
    uint8_t cache_size = 0;
    struct grid_cache *cache = nullptr;

    // a grid_cache block waiting for disk IO
    enum DiskIoState {
        DiskIoIdle      = 0,
        DiskIoWaitWrite = 1,
        DiskIoWaitRead  = 2,
        DiskIoDoneRead  = 3,
        DiskIoDoneWrite = 4
    };
    volatile enum DiskIoState disk_io_state;
    union grid_io_block disk_block;

    // last time we asked for more grids
    uint32_t last_request_time_ms[MAVLINK_COMM_NUM_BUFFERS];

    static const uint64_t bitmap_mask = (((uint64_t)1U)<<(TERRAIN_GRID_BLOCK_MUL_X*TERRAIN_GRID_BLOCK_MUL_Y)) - 1;

    // open file handle on degree file
    int fd;

    // has the timer been setup?
    bool timer_setup;

    // degrees lat and lon of file
    int8_t file_lat_degrees;
    int16_t file_lon_degrees;

    // do we have an IO failure
    volatile bool io_failure;

    // have we created the terrain directory?
    bool directory_created;

    // cache the home altitude, as it is needed so often
    float home_height;
    Location home_loc;

    // cache the last terrain height (AMSL) of the AHRS current
    // location. This is used for extrapolation when terrain data is
    // temporarily unavailable
    bool have_current_loc_height;
    float last_current_loc_height;

    // next mission command to check
    uint16_t next_mission_index;

    // next mission position to check
    uint8_t next_mission_pos;

    // last time the mission changed
    uint32_t last_mission_change_ms;

    // grid spacing during mission check
    uint16_t last_mission_spacing;

    // next rally command to check
    uint16_t next_rally_index;

    // last time the rally points changed
    uint32_t last_rally_change_ms;

    // grid spacing during rally check
    uint16_t last_rally_spacing;

    char *file_path = nullptr;

    // status
    enum TerrainStatus system_status = TerrainStatusDisabled;
};
#endif // AP_TERRAIN_AVAILABLE
