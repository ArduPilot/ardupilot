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
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>

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
    grid.grid.grid_idx_x = info.grid_idx_x;
    grid.grid.grid_idx_y = info.grid_idx_y;
    grid.grid.lat_degrees = info.lat_degrees;
    grid.grid.lon_degrees = info.lon_degrees;
    grid.grid.version = TERRAIN_GRID_FORMAT_VERSION;
    grid.last_access_ms = hal.scheduler->millis();

    // mark as waiting for disk read
    grid.state = GRID_CACHE_DISKWAIT;

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
  request any missing 4x4 grids from a block, given a grid_cache
 */
bool AP_Terrain::request_missing(mavlink_channel_t chan, struct grid_cache &gcache)
{
    struct grid_block &grid = gcache.grid;

    // see if we are waiting for disk read
    if (gcache.state == GRID_CACHE_DISKWAIT) {
        // don't request data from the GCS till we know its not on disk
        return false;
    }

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
  request any missing 4x4 grids from a block
 */
bool AP_Terrain::request_missing(mavlink_channel_t chan, const struct grid_info &info)
{
    // find the grid
    struct grid_cache &gcache = find_grid(info);
    return request_missing(chan, gcache);
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

    // see if we need to schedule some disk IO
    update();

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

    // check cache blocks that may have been setup by a TERRAIN_CHECK
    for (uint16_t i=0; i<TERRAIN_GRID_BLOCK_CACHE_SIZE; i++) {
        if (cache[i].state >= GRID_CACHE_VALID) {
            if (request_missing(chan, cache[i])) {
                return;
            }
        }
    }

    // request the current loc last to ensure it has highest last
    // access time
    if (request_missing(chan, info)) {
        return;
    }

    // nothing to request, send a terrain report
    send_terrain_report(chan, loc);
}

/*
  count bits in a uint64_t
*/
uint8_t AP_Terrain::bitcount64(uint64_t b)
{
    return __builtin_popcount((unsigned)(b&0xFFFFFFFF)) + __builtin_popcount((unsigned)(b>>32));
}

/*
  get some statistics for TERRAIN_REPORT
*/
void AP_Terrain::get_statistics(uint16_t &pending, uint16_t &loaded)
{
    pending = 0;
    loaded = 0;
    for (uint16_t i=0; i<TERRAIN_GRID_BLOCK_CACHE_SIZE; i++) {
        if (cache[i].state == GRID_CACHE_INVALID) {
            continue;
        }
        uint8_t maskbits = TERRAIN_GRID_BLOCK_MUL_X*TERRAIN_GRID_BLOCK_MUL_Y;
        if (cache[i].state == GRID_CACHE_DISKWAIT) {
            pending += maskbits;
            continue;
        }
        uint8_t bitcount = bitcount64(cache[i].grid.bitmap);
        pending += maskbits - bitcount;
        loaded += bitcount;
    }
}


/* 
   handle terrain messages from GCS
 */
void AP_Terrain::handle_data(mavlink_channel_t chan, mavlink_message_t *msg)
{
    if (msg->msgid == MAVLINK_MSG_ID_TERRAIN_DATA) {
        handle_terrain_data(msg);
    } else if (msg->msgid == MAVLINK_MSG_ID_TERRAIN_CHECK) {
        handle_terrain_check(chan, msg);
    }
}


/* 
   send a TERRAIN_REPORT for a location
 */
void AP_Terrain::send_terrain_report(mavlink_channel_t chan, const Location &loc)
{
    float height = 0;
    uint16_t spacing = 0;
    if (height_amsl(loc, height)) {
        spacing = grid_spacing;
    }
    uint16_t pending, loaded;
    get_statistics(pending, loaded);
    if (comm_get_txspace(chan) >= MAVLINK_NUM_NON_PAYLOAD_BYTES + MAVLINK_MSG_ID_TERRAIN_REPORT_LEN) {
        mavlink_msg_terrain_report_send(chan, loc.lat, loc.lng, spacing, height, pending, loaded);
    }
}

/* 
   handle TERRAIN_CHECK messages from GCS
 */
void AP_Terrain::handle_terrain_check(mavlink_channel_t chan, mavlink_message_t *msg)
{
    mavlink_terrain_check_t packet;
    mavlink_msg_terrain_check_decode(msg, &packet);
    Location loc;
    loc.lat = packet.lat;
    loc.lng = packet.lon;
    send_terrain_report(chan, loc);
}

/* 
   handle TERRAIN_DATA messages from GCS
 */
void AP_Terrain::handle_terrain_data(mavlink_message_t *msg)
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
    
    // mark dirty for disk IO
    gcache.state = GRID_CACHE_DIRTY;
    
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
    
    // see if we need to schedule some disk IO
    update();
}

/*
  find cache index of disk_block
 */
int16_t AP_Terrain::find_io_idx(void)
{
    for (uint16_t i=0; i<TERRAIN_GRID_BLOCK_CACHE_SIZE; i++) {
        if (disk_block.block.lat == cache[i].grid.lat &&
            disk_block.block.lon == cache[i].grid.lon) {
            return i;
        }
    }    
    return -1;
}

/*
  check for blocks that need to be read from disk
 */
void AP_Terrain::check_disk_read(void)
{
    for (uint16_t i=0; i<TERRAIN_GRID_BLOCK_CACHE_SIZE; i++) {
        if (cache[i].state == GRID_CACHE_DISKWAIT) {
            disk_block.block = cache[i].grid;
            disk_io_state = DiskIoWaitRead;
            return;
        }
    }    
}

/*
  check for blocks that need to be written to disk
 */
void AP_Terrain::check_disk_write(void)
{
    for (uint16_t i=0; i<TERRAIN_GRID_BLOCK_CACHE_SIZE; i++) {
        if (cache[i].state == GRID_CACHE_DIRTY) {
            disk_block.block = cache[i].grid;
            disk_io_state = DiskIoWaitWrite;
            return;
        }
    }    
}

/*
  update terrain data. Check if we need to do disk IO for grids. This
  should be called at around 1Hz
 */
void AP_Terrain::update(void)
{
    if (enable == 0) {
        return;
    }

    if (!timer_setup) {
        timer_setup = true;
        hal.scheduler->register_io_process(AP_HAL_MEMBERPROC(&AP_Terrain::io_timer));        
    }

    switch (disk_io_state) {
    case DiskIoIdle:
        // look for a block that needs reading or writing
        check_disk_read();
        if (disk_io_state == DiskIoIdle) {
            // still idle, check for writes
            check_disk_write();            
        }
        break;
        
    case DiskIoDoneRead: {
        // a read has completed
        int16_t cache_idx = find_io_idx();
        if (cache_idx != -1) {
            if (disk_block.block.bitmap != 0) {
                // when bitmap is zero we read an empty block
                cache[cache_idx].grid = disk_block.block;
            }
            cache[cache_idx].state = GRID_CACHE_VALID;
            cache[cache_idx].last_access_ms = hal.scheduler->millis();
        }
        disk_io_state = DiskIoIdle;
        break;
    }

    case DiskIoDoneWrite: {
        // a write has completed
        int16_t cache_idx = find_io_idx();
        if (cache_idx != -1) {
            if (cache[cache_idx].grid.bitmap == disk_block.block.bitmap) {
                // only mark valid if more grids haven't been added
                cache[cache_idx].state = GRID_CACHE_VALID;
            }
        }
        disk_io_state = DiskIoIdle;
        break;
    }
        
    case DiskIoWaitWrite:
    case DiskIoWaitRead:
        // waiting for io_timer()
        break;
    }
    
    Location loc;
    if (!ahrs.get_position(loc)) {
        // we don't know where we are
        return;
    }

#if TERRAIN_DEBUG
    static uint32_t last_report_ms;
    float height;
    
    if (hal.scheduler->millis() - last_report_ms > 1000 && height_amsl(loc, height)) {
        printf("height %.2f\n", height);
        last_report_ms = hal.scheduler->millis();
    }
#endif
}


/*
  open the current degree file
 */
void AP_Terrain::open_file(void)
{
    struct grid_block &block = disk_block.block;
    if (fd != -1 && 
        block.lat_degrees == file_lat_degrees &&
        block.lon_degrees == file_lon_degrees) {
        // already open on right file
        return;
    }

    // build the pathname to the degree file
    char path[] = HAL_BOARD_TERRAIN_DIRECTORY "/NxxExxx.DAT";
    char *p = &path[strlen(HAL_BOARD_TERRAIN_DIRECTORY)+1];
    snprintf(p, 12, "%c%02u%c%03u.DAT",
             block.lat_degrees<0?'S':'N',
             abs(block.lat_degrees),
             block.lon_degrees<0?'W':'E',
             abs(block.lon_degrees));

    // create directory if need be
    if (!directory_created) {
        mkdir(HAL_BOARD_TERRAIN_DIRECTORY, 0755);
        directory_created = true;
    }

    if (fd != -1) {
        ::close(fd);
    }
    fd = ::open(path, O_RDWR|O_CREAT, 0644);
    if (fd == -1) {
#if TERRAIN_DEBUG
        hal.console->printf("Open %s failed - %s\n",
                            path, strerror(errno));
#endif
        io_failure = true;
        return;
    }

    file_lat_degrees = block.lat_degrees;
    file_lon_degrees = block.lon_degrees;
}

/*
  seek to the right offset for disk_block
 */
void AP_Terrain::seek_offset(void)
{
    struct grid_block &block = disk_block.block;
    // work out how many longitude blocks there are at this latitude
    Location loc1, loc2;
    loc1.lat = block.lat_degrees*10*1000*1000L;
    loc1.lng = block.lon_degrees*10*1000*1000L;
    loc2.lat = block.lat_degrees*10*1000*1000L;
    loc2.lng = (block.lon_degrees+1)*10*1000*1000L;

    // shift another two blocks east to ensure room is available
    location_offset(loc2, 0, 2*grid_spacing*TERRAIN_GRID_BLOCK_SIZE_Y);
    Vector2f offset = location_diff(loc1, loc2);
    uint16_t east_blocks = offset.y / (grid_spacing*TERRAIN_GRID_BLOCK_SIZE_Y);

    uint32_t file_offset = (east_blocks * block.grid_idx_x + 
                            block.grid_idx_y) * sizeof(union grid_io_block);
    if (::lseek(fd, file_offset, SEEK_SET) != file_offset) {
#if TERRAIN_DEBUG
        hal.console->printf("Seek %lu failed - %s\n",
                            (unsigned long)file_offset, strerror(errno));
#endif
        ::close(fd);
        fd = -1;
        io_failure = true;
    }
}

/*
  write out disk_block
 */
void AP_Terrain::write_block(void)
{
    seek_offset();
    if (io_failure) {
        return;
    }
    ssize_t ret = ::write(fd, &disk_block, sizeof(disk_block));
    if (ret  != sizeof(disk_block)) {
#if TERRAIN_DEBUG
        hal.console->printf("write failed - %s\n", strerror(errno));
#endif
        ::close(fd);
        fd = -1;
        io_failure = true;
    } else {
        ::fsync(fd);
#if TERRAIN_DEBUG
        printf("wrote block at %ld %ld ret=%d mask=%07llx\n",
               (long)disk_block.block.lat,
               (long)disk_block.block.lon,
               (int)ret,
               (unsigned long long)disk_block.block.bitmap);
#endif
    }
    disk_io_state = DiskIoDoneWrite;
}

/*
  read in disk_block
 */
void AP_Terrain::read_block(void)
{
    seek_offset();
    if (io_failure) {
        return;
    }
    int32_t lat = disk_block.block.lat;
    int32_t lon = disk_block.block.lon;

    ssize_t ret = ::read(fd, &disk_block, sizeof(disk_block));
    if (ret != sizeof(disk_block) || 
        disk_block.block.lat != lat || 
        disk_block.block.lon != lon ||
        disk_block.block.bitmap == 0 ||
        disk_block.block.version != TERRAIN_GRID_FORMAT_VERSION) {
#if TERRAIN_DEBUG
        printf("read empty block at %ld %ld ret=%d\n",
               (long)lat,
               (long)lon,
               (int)ret);
#endif
        // a short read or bad data is not an IO failure, just a
        // missing block on disk
        memset(&disk_block, 0, sizeof(disk_block));
        disk_block.block.lat = lat;
        disk_block.block.lon = lon;
        disk_block.block.bitmap = 0;
    } else {
#if TERRAIN_DEBUG
        printf("read block at %ld %ld ret=%d mask=%07llx\n",
               (long)lat,
               (long)lon,
               (int)ret,
               (unsigned long long)disk_block.block.bitmap);
#endif
    }
    disk_io_state = DiskIoDoneRead;
}

/*
  timer called to do disk IO
 */
void AP_Terrain::io_timer(void)
{
    if (io_failure) {
        // don't keep trying io, so we don't thrash the filesystem
        // code while flying
        return;
    }

    switch (disk_io_state) {
    case DiskIoIdle:
    case DiskIoDoneRead:
    case DiskIoDoneWrite:
        // nothing to do
        break;
        
    case DiskIoWaitWrite:
        // need to write out the block
        open_file();
        if (fd == -1) {
            return;
        }
        write_block();
        break;

    case DiskIoWaitRead:
        // need to read in the block
        open_file();
        if (fd == -1) {
            return;
        }
        read_block();
        break;
    }
}

#endif // HAVE_AP_TERRAIN
