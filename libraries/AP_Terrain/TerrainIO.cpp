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
/*
  handle disk IO for terrain code
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
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>

extern const AP_HAL::HAL& hal;

/*
  check for blocks that need to be read from disk
 */
void AP_Terrain::check_disk_read(void)
{
    for (uint16_t i=0; i<cache_size; i++) {
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
    for (uint16_t i=0; i<cache_size; i++) {
        if (cache[i].state == GRID_CACHE_DIRTY) {
            disk_block.block = cache[i].grid;
            disk_io_state = DiskIoWaitWrite;
            return;
        }
    }    
}

/*
  Check if we need to do disk IO for grids. 
 */
void AP_Terrain::schedule_disk_io(void)
{
    if (enable == 0 || !allocate()) {
        return;
    }

    if (!timer_setup) {
        timer_setup = true;
        hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&AP_Terrain::io_timer, void));
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
        int16_t cache_idx = find_io_idx(GRID_CACHE_DISKWAIT);
        if (cache_idx != -1) {
            if (disk_block.block.bitmap != 0) {
                // when bitmap is zero we read an empty block
                cache[cache_idx].grid = disk_block.block;
            }
            cache[cache_idx].state = GRID_CACHE_VALID;
            cache[cache_idx].last_access_ms = AP_HAL::millis();
        }
        disk_io_state = DiskIoIdle;
        break;
    }

    case DiskIoDoneWrite: {
        // a write has completed
        int16_t cache_idx = find_io_idx(GRID_CACHE_DIRTY);
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
}


/********************************************************
All the functions below this point run in the IO timer context, which
is a separate thread. The code uses the state machine controlled by
disk_io_state to manage who has access to the structures and to
prevent race conditions.

The IO timer context owns the data when disk_io_state is
DiskIoWaitWrite or DiskIoWaitRead. The main thread owns the data when
disk_io_state is DiskIoIdle, DiskIoDoneWrite or DiskIoDoneRead

All file operations are done by the IO thread.
*********************************************************/


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
    if (file_path == NULL) {
        const char* terrain_dir = hal.util->get_custom_terrain_directory();
        if (terrain_dir == NULL) {
            terrain_dir = HAL_BOARD_TERRAIN_DIRECTORY;
        }
        if (asprintf(&file_path, "%s/NxxExxx.DAT", terrain_dir) <= 0) {
            io_failure = true;
            file_path = NULL;
            return;
        }
    }
    if (file_path == NULL) {
        io_failure = true;
        return;
    }
    char *p = &file_path[strlen(file_path)-12];
    if (*p != '/') {
        io_failure = true;
        return;        
    }
    snprintf(p, 13, "/%c%02u%c%03u.DAT",
             block.lat_degrees<0?'S':'N',
             abs(block.lat_degrees),
             block.lon_degrees<0?'W':'E',
             abs(block.lon_degrees));

    // create directory if need be
    if (!directory_created) {
        *p = 0;
        mkdir(file_path, 0755);
        directory_created = true;
        *p = '/';
    }

    if (fd != -1) {
        ::close(fd);
    }
    fd = ::open(file_path, O_RDWR|O_CREAT, 0644);
    if (fd == -1) {
#if TERRAIN_DEBUG
        hal.console->printf("Open %s failed - %s\n",
                            file_path, strerror(errno));
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
    if (::lseek(fd, file_offset, SEEK_SET) != (off_t)file_offset) {
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

    disk_block.block.crc = get_block_crc(disk_block.block);

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
        disk_block.block.spacing != grid_spacing ||
        disk_block.block.version != TERRAIN_GRID_FORMAT_VERSION ||
        disk_block.block.crc != get_block_crc(disk_block.block)) {
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

#endif // AP_TERRAIN_AVAILABLE
