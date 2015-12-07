
#include <string.h>
#include "Storage.h"

using namespace Empty;

Storage::Storage()
{}

void Storage::init()
{}

void Storage::read_block(void* dst, uint16_t src, size_t n) {
    memset(dst, 0, n);
}

void Storage::write_block(uint16_t loc, const void* src, size_t n)
{}

