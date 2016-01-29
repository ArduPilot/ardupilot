#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_QURT

#include <assert.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include "Storage.h"

using namespace QURT;

/*
  This stores 'eeprom' data on the filesystem, with a 16k size
  
  Data is written on the ARM frontend via a RPC call
 */

extern const AP_HAL::HAL& hal;

volatile bool Storage::dirty;
uint8_t Storage::buffer[QURT_STORAGE_SIZE];
Semaphore Storage::lock;

void Storage::read_block(void *dst, uint16_t loc, size_t n) 
{
    if (loc >= sizeof(buffer)-(n-1)) {
        return;
    }
    memcpy(dst, &buffer[loc], n);
}

void Storage::write_block(uint16_t loc, const void *src, size_t n) 
{
    if (loc >= sizeof(buffer)-(n-1)) {
        return;
    }
    if (memcmp(src, &buffer[loc], n) != 0) {
        lock.take(0);
        memcpy(&buffer[loc], src, n);
        dirty = true;
        lock.give();
    }
}

#endif // CONFIG_HAL_BOARD
