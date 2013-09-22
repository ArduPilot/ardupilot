
#include <string.h>
#include "Storage.h"

using namespace Linux;

LinuxStorage::LinuxStorage()
{}

void LinuxStorage::init(void*)
{}

uint8_t LinuxStorage::read_byte(uint16_t loc){
    return 0;
}

uint16_t LinuxStorage::read_word(uint16_t loc){
    return 0;
}

uint32_t LinuxStorage::read_dword(uint16_t loc){
    return 0;
}

void LinuxStorage::read_block(void* dst, uint16_t src, size_t n) {
    memset(dst, 0, n);
}

void LinuxStorage::write_byte(uint16_t loc, uint8_t value)
{}

void LinuxStorage::write_word(uint16_t loc, uint16_t value)
{}

void LinuxStorage::write_dword(uint16_t loc, uint32_t value)
{}

void LinuxStorage::write_block(uint16_t loc, const void* src, size_t n)
{}

