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
  handle vehicle <-> GCS communications for terrain library
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

extern const AP_HAL::HAL& hal;

/*
  request any missing 4x4 grids from a block, given a grid_cache
 */
bool AP_Terrain::request_missing(mavlink_channel_t chan, struct grid_cache &gcache)
{
    struct grid_block &grid = gcache.grid;

    if (grid.spacing != grid_spacing) {
        // an invalid grid
        return false;
    }

    // see if we are waiting for disk read
    if (gcache.state == GRID_CACHE_DISKWAIT) {
        // don't request data from the GCS till we know it's not on disk
        return false;
    }

    // see if it is fully populated
    if ((grid.bitmap & bitmap_mask) == bitmap_mask) {
        // it is fully populated, nothing to do
        return false;
    }

    if (!HAVE_PAYLOAD_SPACE(chan, TERRAIN_REQUEST)) {
        // not enough buffer space
        return false;
    }

    /*
      ask the GCS to send a set of 4x4 grids
     */
    mavlink_msg_terrain_request_send(chan, grid.lat, grid.lon, grid_spacing, bitmap_mask & ~grid.bitmap);
    last_request_time_ms[chan] = AP_HAL::millis();

    return true;
}

/*
  request any missing 4x4 grids from a block
 */
bool AP_Terrain::request_missing(mavlink_channel_t chan, const struct grid_info &info)
{
    // find the grid
    struct grid_cache &gcache = find_grid_cache(info);
    return request_missing(chan, gcache);
}

/*
  send any pending terrain request to the GCS
 */
void AP_Terrain::send_request(mavlink_channel_t chan)
{
    if (!allocate()) {
        // not enabled
        return;
    }

    // see if we need to schedule some disk IO
    schedule_disk_io();

    Location loc;
    if (!AP::ahrs().get_position(loc)) {
        // we don't know where we are
        return;
    }

    // always send a terrain report
    send_terrain_report(chan, loc, true);

    // did we request recently?
    if (AP_HAL::millis() - last_request_time_ms[chan] < 2000) {
        // too soon to request again
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
    for (uint16_t i=0; i<cache_size; i++) {
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
    for (uint16_t i=0; i<cache_size; i++) {
        if (cache[i].grid.spacing != grid_spacing) {
            continue;
        }
        if (cache[i].state == GRID_CACHE_INVALID) {
            continue;
        }
        uint8_t maskbits = TERRAIN_GRID_BLOCK_MUL_X*TERRAIN_GRID_BLOCK_MUL_Y;
        if (cache[i].state == GRID_CACHE_DISKWAIT) {
            pending += maskbits;
            continue;
        }
        if (cache[i].state == GRID_CACHE_DIRTY) {
            // count dirty grids as a pending, so we know where there 
            // are disk writes pending
            pending++;
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
void AP_Terrain::send_terrain_report(mavlink_channel_t chan, const Location &loc, bool extrapolate)
{
    float terrain_height = 0;
    float home_terrain_height = 0;
    uint16_t spacing = 0;
    Location current_loc;
    const AP_AHRS &ahrs = AP::ahrs();
    if (ahrs.get_position(current_loc) &&
        height_amsl(ahrs.get_home(), home_terrain_height, false) &&
        height_amsl(loc, terrain_height, false)) {
        // non-zero spacing indicates we have data
        spacing = grid_spacing;
    } else if (extrapolate && have_current_loc_height) {
        // show the extrapolated height, so logs show what height is
        // being used for navigation
        terrain_height = last_current_loc_height;
    }
    uint16_t pending, loaded;
    get_statistics(pending, loaded);

    float current_height;
    if (spacing == 0 && !(extrapolate && have_current_loc_height)) {
        current_height = 0;
    } else {
        if (current_loc.relative_alt) {
            current_height = current_loc.alt*0.01f;
        } else {
            current_height = (current_loc.alt - ahrs.get_home().alt)*0.01f;
        }
    }
    current_height += home_terrain_height - terrain_height;

    if (HAVE_PAYLOAD_SPACE(chan, TERRAIN_REPORT)) {
        mavlink_msg_terrain_report_send(chan, loc.lat, loc.lng, spacing, 
                                        terrain_height, current_height,
                                        pending, loaded);
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
    send_terrain_report(chan, loc, false);
}

/* 
   handle TERRAIN_DATA messages from GCS
 */
void AP_Terrain::handle_terrain_data(mavlink_message_t *msg)
{
    mavlink_terrain_data_t packet;
    mavlink_msg_terrain_data_decode(msg, &packet);

    uint16_t i;
    for (i=0; i<cache_size; i++) {
        if (cache[i].grid.lat == packet.lat && 
            cache[i].grid.lon == packet.lon && 
            cache[i].grid.spacing == packet.grid_spacing &&
            grid_spacing == packet.grid_spacing &&
            packet.gridbit < 56) {
            break;
        }
    }
    if (i == cache_size) {
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


#endif // AP_TERRAIN_AVAILABLE
