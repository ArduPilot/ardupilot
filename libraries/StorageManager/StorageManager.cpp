/*
   Please contribute your ideas! See https://ardupilot.org/dev for details

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
  Management for hal.storage to allow for backwards compatible mapping
  of storage offsets to available storage
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include <GCS_MAVLink/GCS.h>

#include "StorageManager.h"

#include <stdio.h>

extern const AP_HAL::HAL& hal;

bool StorageManager::last_io_failed;

/*
  the layouts below are carefully designed to ensure backwards
  compatibility with older firmwares
 */

#if STORAGE_NUM_AREAS == 1
/*
  layout for peripherals
 */
const StorageManager::StorageArea StorageManager::layout[STORAGE_NUM_AREAS] = {
    { StorageParam,   0,     HAL_STORAGE_SIZE}
};

#else
/*
  layout for fixed wing and rovers
  On PX4v1 this gives 309 waypoints, 30 rally points and 52 fence points
  On Pixhawk this gives 724 waypoints, 50 rally points and 84 fence points
 */
/*
  layout for copter.
  On PX4v1 this gives 303 waypoints, 26 rally points and 38 fence points
  On Pixhawk this gives 718 waypoints, 46 rally points and 70 fence points
 */
const StorageManager::StorageArea StorageManager::layout[STORAGE_NUM_AREAS] = {
#if !APM_BUILD_COPTER_OR_HELI
    { StorageParam,   0,     1280}, // 0x500 parameter bytes
    { StorageMission, 1280,  2506},
    { StorageRally,   3786,   150}, // 10 rally points
    { StorageFence,   3936,   160}, // 20 fence points
#else
    { StorageParam,   0,     1536}, // 0x600 param bytes
    { StorageMission, 1536,  2422},
    { StorageRally,   3958,    90}, // 6 rally points
    { StorageFence,   4048,    48}, // 6 fence points
#endif
#if STORAGE_NUM_AREAS >= 10
    { StorageParam,   4096,  1280},
    { StorageRally,   5376,   300},
    { StorageFence,   5676,   256},
    { StorageMission, 5932,  2132},
    { StorageKeys,    8064,    64}, 
    { StorageBindInfo,8128,    56},
#endif
#if STORAGE_NUM_AREAS == 11
    // optimised for lots of parameters for 15k boards with OSD
    { StorageParam,    8192,  7168},
#elif STORAGE_NUM_AREAS == 12
    // optimised for lots of parameters for 15k boards with OSD, plus room for CAN DNA
    { StorageParam,    8192,  6144},
    { StorageCANDNA,   14336, 1024},
#endif
#if STORAGE_NUM_AREAS >= 15
    { StorageParam,    8192,  1280},
    { StorageRally,    9472,   300},
    { StorageFence,    9772,   256},
    { StorageMission,  10028,  5204}, // leave 128 byte gap for expansion
    { StorageCANDNA,   15232,  1024},
    // 128 byte gap at end of first 16k
#endif
#if STORAGE_NUM_AREAS >= 18
    { StorageParam,    16384, 1280},
    { StorageMission,  17664, 9842},
    { StorageParamBak, 27506, 5262},
#endif
};
#endif // STORAGE_NUM_AREAS == 1

/*
  erase all storage
 */
void StorageManager::erase(void)
{
    if (!hal.storage->erase()) {
        ::printf("StorageManager: erase failed\n");
    }
}

/*
  constructor for StorageAccess
 */
StorageAccess::StorageAccess(StorageManager::StorageType _type) : 
    type(_type) 
{
    // calculate available bytes
    total_size = 0;
#if AP_SDCARD_STORAGE_ENABLED
    file = nullptr;
#endif
    for (uint8_t i=0; i<STORAGE_NUM_AREAS; i++) {
        const StorageManager::StorageArea &area = StorageManager::layout[i];
        if (area.type == type) {
            total_size += area.length;
        }
    }
}

/*
  base read function. The src offset is within the bytes allocated
  for the storage type of this StorageAccess object
*/
bool StorageAccess::read_block(void *data, uint16_t addr, size_t n) const
{
    uint8_t *b = (uint8_t *)data;

#if AP_SDCARD_STORAGE_ENABLED
    if (file != nullptr) {
        // using microSD data
        if (addr > file->bufsize) {
            return false;
        }
        const size_t n2 = MIN(n, file->bufsize - addr);
        memcpy(b, &file->buffer[addr], n2);
        return n == n2;
    }
#endif

    for (uint8_t i=0; i<STORAGE_NUM_AREAS; i++) {
        const StorageManager::StorageArea &area = StorageManager::layout[i];
        uint16_t length = area.length;
        uint16_t offset = area.offset;
        if (area.type != type) {
            continue;
        }
        if (addr >= length) {
            // the data isn't in this area
            addr -= length;
            continue;
        }
        uint16_t count = n;
        if (count+addr > length) {
            // the data crosses a boundary between two areas
            count = length - addr;
        }
        hal.storage->read_block(b, addr+offset, count);
        n -= count;

        if (n == 0) {
            break;
        }

        // move pointer after written bytes
        b += count;
        // continue writing at the beginning of next valid area
        addr = 0;
    }

    return (n == 0);
}


/*
  base write function. The addr offset is within the bytes allocated
  for the storage type of this StorageAccess object
*/
bool StorageAccess::write_block(uint16_t addr, const void *data, size_t n) const
{
    const uint8_t *b = (const uint8_t *)data;

#if AP_SDCARD_STORAGE_ENABLED
    if (file != nullptr) {
        if (addr > file->bufsize) {
            return false;
        }
        // using microSD data
        WITH_SEMAPHORE(file->sem);
        const size_t n2 = MIN(n, file->bufsize - addr);
        memcpy(&file->buffer[addr], b, n2);
        for (uint8_t i=addr/1024U; i<(addr+n2+1023U)/1024U; i++) {
            file->dirty_mask |= (1ULL<<i);
        }
        return n == n2;
    }
#endif

    for (uint8_t i=0; i<STORAGE_NUM_AREAS; i++) {
        const StorageManager::StorageArea &area = StorageManager::layout[i];
        uint16_t length = area.length;
        uint16_t offset = area.offset;
        if (area.type != type) {
            continue;
        }
        if (addr >= length) {
            // the data isn't in this area
            addr -= length;
            continue;
        }
        uint16_t count = n;
        if (count+addr > length) {
            // the data crosses a boundary between two areas
            count = length - addr;
        }
        hal.storage->write_block(addr+offset, b, count);
        n -= count;

        if (n == 0) {
            break;
        }

        // move pointer after written bytes
        b += count;
        // continue writing at the beginning of next valid area
        addr = 0;
    }

    return (n == 0);
}

/*
  read a byte
 */
uint8_t StorageAccess::read_byte(uint16_t loc) const
{
    uint8_t v;
    read_block(&v, loc, sizeof(v));
    return v;
}

/*
  read 16 bit value
 */
uint16_t StorageAccess::read_uint16(uint16_t loc) const
{
    uint16_t v;
    read_block(&v, loc, sizeof(v));
    return v;
}

/*
  read 32 bit value
 */
uint32_t StorageAccess::read_uint32(uint16_t loc) const
{
    uint32_t v;
    read_block(&v, loc, sizeof(v));
    return v;
}

/*
  read a float
 */
float StorageAccess::read_float(uint16_t loc) const
{
    float v;
    read_block(&v, loc, sizeof(v));
    return v;
}

/*
  write a byte
 */
void StorageAccess::write_byte(uint16_t loc, uint8_t value) const
{
    write_block(loc, &value, sizeof(value));
}

/*
  write a uint16
 */
void StorageAccess::write_uint16(uint16_t loc, uint16_t value) const
{
    write_block(loc, &value, sizeof(value));
}

/*
  write a uint32
 */
void StorageAccess::write_uint32(uint16_t loc, uint32_t value) const
{
    write_block(loc, &value, sizeof(value));
}

/*
  write a float
 */
void StorageAccess::write_float(uint16_t loc, float value) const
{
    write_block(loc, &value, sizeof(value));
}

/*
  copy one area to another
 */
bool StorageAccess::copy_area(const StorageAccess &source) const
{
    // we deliberately allow for copies from smaller areas. This
    // allows for a partial backup region for parameters
    uint16_t total = MIN(source.size(), size());
    uint16_t ofs = 0;
    while (total > 0) {
        uint8_t block[32];
        uint16_t n = MIN(sizeof(block), total);
        if (!source.read_block(block, ofs, n) ||
            !write_block(ofs, block, n)) {
            return false;
        }
        total -= n;
        ofs += n;
    }
    return true;
}

#if AP_SDCARD_STORAGE_ENABLED
/*
  attach a file to a storage region
 */
bool StorageAccess::attach_file(const char *filename, uint16_t size_kbyte)
{
    if (file != nullptr) {
        // only one attach per boot
        return false;
    }
    const uint32_t size = MIN(0xFFFFU, size_kbyte * 1024U);
    auto *newfile = new FileStorage;
    if (newfile == nullptr) {
        AP_BoardConfig::allocation_error("StorageFile");
    }
    ssize_t nread;

    newfile->fd = AP::FS().open(filename, O_RDWR | O_CREAT);
    if (newfile->fd == -1) {
        goto fail;
    }
    newfile->buffer = new uint8_t[size];
    if (newfile->buffer == nullptr) {
        AP_BoardConfig::allocation_error("StorageFile");
    }
    newfile->bufsize = size;
    nread = AP::FS().read(newfile->fd, newfile->buffer, size);
    if (nread == -1) {
        goto fail;
    }
    if (nread == 0) {
        // new file, copy storage from existing to allow users to
        // start with existing mission
        read_block(newfile->buffer, 0, total_size);
    }
    if (nread < int32_t(size)) {
        if (AP::FS().write(newfile->fd, &newfile->buffer[nread], size-nread) != int32_t(size-nread)) {
            goto fail;
        }
        if (AP::FS().fsync(newfile->fd) != 0) {
            goto fail;
        }
    }

    hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&StorageAccess::flush_file, void));

    file = newfile;
    total_size = newfile->bufsize;

    return true;

fail:
    if (newfile->fd != -1) {
        AP::FS().close(newfile->fd);
    }
    if (newfile->buffer != nullptr) {
        delete[] newfile->buffer;
    }
    delete newfile;
    return false;
}

/*
  flush file changes to microSD
 */
void StorageAccess::flush_file(void)
{
    if (file == nullptr || file->dirty_mask == 0) {
        return;
    }
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - file->last_clean_ms < 1000U) {
        return;
    }
    WITH_SEMAPHORE(file->sem);

    if (StorageManager::last_io_failed &&
        now_ms - file->last_io_fail_ms < 2000U) {
        // don't retry too fast
        return;
    }

    // write out 1k at a time
    bool io_fail = false;
    const int b = __builtin_ffsll(file->dirty_mask);
    const uint32_t ofs = (b-1)*1024;
    const uint32_t len = MIN(1024U, file->bufsize-ofs);
    if (AP::FS().lseek(file->fd, ofs, SEEK_SET) != int32_t(ofs) ||
        AP::FS().write(file->fd, &file->buffer[ofs], len) != int32_t(len)) {
        io_fail = true;
    } else {
        file->dirty_mask &= ~(1ULL<<(b-1));
    }
    if (file->dirty_mask == 0) {
        file->last_clean_ms = now_ms;
        if (AP::FS().fsync(file->fd) != 0) {
            io_fail = true;
        }
    }
    if (io_fail && !StorageManager::last_io_failed) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Mission storage failed");
    } else if (!io_fail && StorageManager::last_io_failed) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Mission storage OK");
    }
    StorageManager::last_io_failed = io_fail;
    if (io_fail) {
        file->last_io_fail_ms = now_ms;
    }
}
#endif // AP_SDCARD_STORAGE_ENABLED

