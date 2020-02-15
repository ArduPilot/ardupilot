#include "AP_HAL.h"
#include "Storage.h"
#include <AP_Math/AP_Math.h>

/*
  default erase method
 */
bool AP_HAL::Storage::erase(void)
{
    uint8_t blk[16] {};
    uint32_t ofs;
    for (ofs=0; ofs<HAL_STORAGE_SIZE; ofs += sizeof(blk)) {
        uint32_t n = MIN(sizeof(blk), HAL_STORAGE_SIZE - ofs);
        write_block(ofs, blk, n);
    }
    return true;
}
