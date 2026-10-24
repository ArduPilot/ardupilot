// SPDX-License-Identifier: GPL-3.0-or-later
#include <stddef.h>
#include <new>

// The module runtime implements these operators with the worker's host allocator.
extern "C" void *aerobatics_allocate(size_t bytes)
{
    return ::operator new(bytes, std::nothrow);
}

extern "C" void aerobatics_free(void *memory)
{
    ::operator delete(memory);
}
