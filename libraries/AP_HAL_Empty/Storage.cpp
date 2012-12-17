
#include <string.h>
#include "Storage.h"

using namespace Empty;

EmptyStorage::EmptyStorage()
{}

void EmptyStorage::init(void*)
{}

uint8_t EmptyStorage::read_byte(uint16_t loc){
    return 0;
}

uint16_t EmptyStorage::read_word(uint16_t loc){
    return 0;
}

uint32_t EmptyStorage::read_dword(uint16_t loc){
    return 0;
}

void EmptyStorage::read_block(void* dst, uint16_t src, size_t n) {
    memset(dst, 0, n);
}

void EmptyStorage::write_byte(uint16_t loc, uint8_t value)
{}

void EmptyStorage::write_word(uint16_t loc, uint16_t value)
{}

void EmptyStorage::write_dword(uint16_t loc, uint32_t value)
{}

void EmptyStorage::write_block(uint16_t loc, void* src, size_t n)
{}

