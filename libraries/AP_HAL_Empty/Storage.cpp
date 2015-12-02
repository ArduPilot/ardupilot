
#include <string.h>
#include "Storage.h"

using namespace Empty;

EmptyStorage::EmptyStorage()
{}

void EmptyStorage::init(void*)
{}

void EmptyStorage::read_block(void* dst, uint16_t src, size_t n) {
    memset(dst, 0, n);
}

void EmptyStorage::write_block(uint16_t loc, const void* src, size_t n)
{}

