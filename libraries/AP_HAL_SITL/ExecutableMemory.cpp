// SPDX-License-Identifier: GPL-3.0-or-later
#include "Util.h"

#if HAL_EXECUTABLE_MEMORY_ENABLED
#include <AP_HAL/utility/ExecutableMemory_posix.h>

AP_HAL::ExecutableMemory::Target HALSITL::Util::executable_memory_target() const
{
#if defined(__x86_64__) && !defined(__ILP32__) && !defined(__CYGWIN__)
    return AP_HAL::ExecutableMemory::Target::X86_64Baseline;
#else
    return AP_HAL::ExecutableMemory::Target::Unsupported;
#endif
}

AP_HAL::ExecutableMemory *HALSITL::Util::allocate_executable(uint32_t size, uint32_t alignment)
{
    return ExecutableMemoryPosix::create(size, alignment);
}

bool HALSITL::Util::is_executable(const void *address, uint32_t size) const
{
    // Host ROMFS is ordinary process data. Keep x86 modules in mmap-backed
    // storage so publish() can apply W^X permissions to the copied code.
    (void)address;
    (void)size;
    return false;
}
#endif
