// SPDX-License-Identifier: GPL-3.0-or-later
#include "ExecutableMemory_posix.h"

#if HAL_EXECUTABLE_MEMORY_ENABLED && (CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX)
#include <sys/mman.h>
#include <unistd.h>

AP_HAL::ExecutableMemory *ExecutableMemoryPosix::create(uint32_t size, uint32_t alignment)
{
    // Module RX/RW boundaries are 4 KiB (x86-64) or 16 KiB (AArch64).
    // publish() refuses a boundary that is not on a host page.
    const long page = sysconf(_SC_PAGESIZE);
    if (size == 0 || size > 16U * 1024 * 1024 || (page != 4096 && page != 16384) ||
        (alignment != 0 && (alignment < sizeof(uintptr_t) || alignment > 4096 || (alignment & (alignment - 1)) != 0))) {
        return nullptr;
    }
    size = (size + uint32_t(page) - 1) & ~(uint32_t(page) - 1);
    void *data = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (data == MAP_FAILED) {
        return nullptr;
    }
    auto *memory = NEW_NOTHROW ExecutableMemoryPosix(data, size);
    if (memory == nullptr) {
        munmap(data, size);
    }
    return memory;
}

ExecutableMemoryPosix::~ExecutableMemoryPosix()
{
    munmap(_data, _size);
}

bool ExecutableMemoryPosix::publish(uint32_t code_size)
{
    if (_published || code_size == 0 || code_size > _size || code_size % uint32_t(sysconf(_SC_PAGESIZE)) != 0) {
        return false;
    }
    __builtin___clear_cache(static_cast<char *>(_data), static_cast<char *>(_data) + code_size);
    if (mprotect(_data, code_size, PROT_READ | PROT_EXEC) != 0) {
        return false;
    }
    _published = true;
    return true;
}
#endif
