/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   Please contribute your ideas! See http://dev.ardupilot.com for details

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

#include <AP_HAL.h>
#include <StorageManager.h>

extern const AP_HAL::HAL& hal;

/*
  the layouts below are carefully designed to ensure backwards
  compatibility with older firmwares
 */

/*
  layout for fixed wing and rovers
  On APM2 this gives 167 waypoints, 10 rally points and 20 fence points
  On PX4v1 this gives 309 waypoints, 30 rally points and 52 fence points
  On Pixhawk this gives 724 waypoints, 50 rally points and 84 fence points
 */
const StorageManager::StorageArea StorageManager::layout_default[STORAGE_NUM_AREAS] PROGMEM = {
    { StorageParam,   0,     1280}, // 0x500 parameter bytes
    { StorageMission, 1280,  2506},
    { StorageRally,   3786,   150}, // 10 rally points
    { StorageFence,   3936,   160}, // 20 fence points
#if STORAGE_NUM_AREAS >= 8
    { StorageParam,   4096,  1280},
    { StorageRally,   5376,   300},
    { StorageFence,   5676,   256},
    { StorageMission, 5932,  2132}, // leave 4 byte gap for PX4
                                    // sentinal and expansion
#endif
#if STORAGE_NUM_AREAS >= 12
    { StorageParam,    8192,  1280},
    { StorageRally,    9472,   300},
    { StorageFence,    9772,   256},
    { StorageMission, 10028,  6228}, // leave 128 byte gap for expansion
#endif
};


/*
  layout for copter. 
  On APM2 this gives 161 waypoints, 6 fence points and 6 rally points
  On PX4v1 this gives 303 waypoints, 26 rally points and 38 fence points
  On Pixhawk this gives 718 waypoints, 46 rally points and 70 fence points
 */
const StorageManager::StorageArea StorageManager::layout_copter[STORAGE_NUM_AREAS] PROGMEM = {
    { StorageParam,   0,     1536}, // 0x600 param bytes
    { StorageMission, 1536,  2422},
    { StorageRally,   3958,    90}, // 6 rally points
    { StorageFence,   4048,    48}, // 6 fence points
#if STORAGE_NUM_AREAS >= 8
    { StorageParam,   4096,  1280},
    { StorageRally,   5376,   300},
    { StorageFence,   5676,   256},
    { StorageMission, 5932,  2132}, // leave 128 byte gap for
                                    // expansion and PX4 sentinal
#endif
#if STORAGE_NUM_AREAS >= 12
    { StorageParam,    8192,  1280},
    { StorageRally,    9472,   300},
    { StorageFence,    9772,   256},
    { StorageMission, 10028,  6228}, // leave 128 byte gap for expansion
#endif
};

// setup default layout
const StorageManager::StorageArea *StorageManager::layout = layout_default;

/*
  erase all storage
 */
void StorageManager::erase(void)
{
    uint8_t blk[16];
    memset(blk, 0, sizeof(blk));
    for (uint8_t i=0; i<STORAGE_NUM_AREAS; i++) {
        const StorageManager::StorageArea &area = StorageManager::layout[i];
        uint16_t length = pgm_read_word(&area.length);
        uint16_t offset = pgm_read_word(&area.offset);
        for (uint8_t ofs=0; length; ofs += sizeof(blk)) {
            uint8_t n = 16;
            if (ofs + n > length) {
                n = length - ofs;
            }
            hal.storage->write_block(offset + ofs, blk, n);
        }
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
    for (uint8_t i=0; i<STORAGE_NUM_AREAS; i++) {
        const StorageManager::StorageArea &area = StorageManager::layout[i];
        if (pgm_read_byte(&area.type) == type) {
            total_size += pgm_read_word(&area.length);
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
    for (uint8_t i=0; i<STORAGE_NUM_AREAS; i++) {
        const StorageManager::StorageArea &area = StorageManager::layout[i];
        uint16_t length = pgm_read_word(&area.length);
        uint16_t offset = pgm_read_word(&area.offset);
        if (pgm_read_byte(&area.type) != type) {
            continue;
        }
        if (addr >= length) {
            // the data isn't in this area
            addr -= length;
            continue;
        }
        uint8_t count = n;
        if (count+addr > length) {
            // the data crosses a boundary between two areas
            count = length - addr;
        }
        hal.storage->read_block(b, addr+offset, count);
        n -= count;
        addr += count;
        b += count;

        // adjust addr for next area
        addr -= length;

        if (n == 0) {
            break;
        }
    }
    return (n == 0);
}


/*
  base read function. The addr offset is within the bytes allocated
  for the storage type of this StorageAccess object
*/
bool StorageAccess::write_block(uint16_t addr, const void *data, size_t n) const
{
    const uint8_t *b = (const uint8_t *)data;
    for (uint8_t i=0; i<STORAGE_NUM_AREAS; i++) {
        const StorageManager::StorageArea &area = StorageManager::layout[i];
        uint16_t length = pgm_read_word(&area.length);
        uint16_t offset = pgm_read_word(&area.offset);
        if (pgm_read_byte(&area.type) != type) {
            continue;
        }
        if (addr >= length) {
            // the data isn't in this area
            addr -= length;
            continue;
        }
        uint8_t count = n;
        if (count+addr > length) {
            // the data crosses a boundary between two areas
            count = length - addr;
        }
        hal.storage->write_block(addr+offset, b, count);
        n -= count;
        addr += count;
        b += count;

        // adjust addr for next area
        addr -= length;

        if (n == 0) {
            break;
        }
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
