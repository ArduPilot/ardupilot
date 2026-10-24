// SPDX-License-Identifier: GPL-3.0-or-later
#include "ExecutableMemory_posix.h"

#if HAL_EXECUTABLE_MEMORY_ENABLED && (CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX)
#include <sys/mman.h>
#include <unistd.h>

AP_HAL::ExecutableMemory *ExecutableMemoryPosix::create(uint32_t size, uint32_t alignment)
{
    // The initial module profiles have 4 KiB permission boundaries. Refuse
    // incompatible host page sizes rather than sharing code and data pages.
    if (size == 0 || size > 16U * 1024 * 1024 || sysconf(_SC_PAGESIZE) != 4096 ||
        (alignment != 0 && (alignment < sizeof(uintptr_t) || alignment > 4096 || (alignment & (alignment - 1)) != 0))) {
        return nullptr;
    }
    size = (size + 4095) & ~4095U;
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
    if (_published || code_size == 0 || code_size > _size || code_size % 4096 != 0) {
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
