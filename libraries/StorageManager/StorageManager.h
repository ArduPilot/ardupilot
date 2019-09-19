/*
   Please contribute your ideas! See http://dev.ardupilot.org for details

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
#pragma once

#include <AP_HAL/AP_HAL.h>

/*
  use just one area per storage type for boards with 4k of
  storage. Use larger areas for other boards
 */
#if HAL_STORAGE_SIZE >= 16384
#define STORAGE_NUM_AREAS 15
#elif HAL_STORAGE_SIZE >= 15360
#define STORAGE_NUM_AREAS 11
#elif HAL_STORAGE_SIZE >= 8192
#define STORAGE_NUM_AREAS 10
#elif HAL_STORAGE_SIZE >= 4096
#define STORAGE_NUM_AREAS 4
#elif HAL_STORAGE_SIZE > 0
#define STORAGE_NUM_AREAS 1
#else
#error "Unsupported storage size"
#endif

/*
  The StorageManager holds the layout of non-volatile storeage
 */
class StorageManager {
    friend class StorageAccess;
public:
    enum StorageType {
        StorageParam   = 0,
        StorageFence   = 1,
        StorageRally   = 2,
        StorageMission = 3,
        StorageKeys    = 4,
        StorageBindInfo= 5,
        StorageCANDNA  = 6
    };

    // erase whole of storage
    static void erase(void);

    // setup for copter layout of storage
    static void set_layout_copter(void) { layout = layout_copter; }

private:
    struct StorageArea {
        StorageType type;
        uint16_t    offset;
        uint16_t    length;
    };

    // available layouts
    static const StorageArea layout_copter[STORAGE_NUM_AREAS];
    static const StorageArea layout_default[STORAGE_NUM_AREAS];
    static const StorageArea *layout;
};

/*
  A StorageAccess object allows access to one type of storage
 */
class StorageAccess {
public:
    // constructor
    StorageAccess(StorageManager::StorageType _type);

    // return total size of this accessor
    uint16_t size(void) const { return total_size; }

    // base access via block functions
    bool read_block(void *dst, uint16_t src, size_t n) const;
    bool write_block(uint16_t dst, const void* src, size_t n) const;    

    // helper functions
    uint8_t  read_byte(uint16_t loc) const;
    uint8_t  read_uint8(uint16_t loc) const { return read_byte(loc); }
    uint16_t read_uint16(uint16_t loc) const;
    uint32_t read_uint32(uint16_t loc) const;

    void write_byte(uint16_t loc, uint8_t value) const;
    void write_uint8(uint16_t loc, uint8_t value) const { return write_byte(loc, value); }
    void write_uint16(uint16_t loc, uint16_t value) const;
    void write_uint32(uint16_t loc, uint32_t value) const;

private:
    const StorageManager::StorageType type;
    uint16_t total_size;
};
