#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_YUNEEC

#include <string.h>
#include "Storage.h"

using namespace YUNEEC;

YUNEECStorage::YUNEECStorage()
{}

void YUNEECStorage::init(void*)
{}

uint8_t YUNEECStorage::read_byte(uint16_t loc){
    return 0;
}

uint16_t YUNEECStorage::read_word(uint16_t loc){
    return 0;
}

uint32_t YUNEECStorage::read_dword(uint16_t loc){
    return 0;
}

void YUNEECStorage::read_block(void* dst, uint16_t src, size_t n) {
    memset(dst, 0, n);
}

void YUNEECStorage::write_byte(uint16_t loc, uint8_t value)
{}

void YUNEECStorage::write_word(uint16_t loc, uint16_t value)
{}

void YUNEECStorage::write_dword(uint16_t loc, uint32_t value)
{}

void YUNEECStorage::write_block(uint16_t loc, const void* src, size_t n)
{}

#endif
