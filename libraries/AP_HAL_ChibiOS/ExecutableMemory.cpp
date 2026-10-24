// SPDX-License-Identifier: GPL-3.0-or-later
#include <AP_HAL/AP_HAL_Boards.h>

#if HAL_EXECUTABLE_MEMORY_ENABLED
#include <hal.h>
#include "Util.h"
#include "hwdef/common/stm32_util.h"

extern "C" {
extern const uint8_t __firmware_crc_start__;
extern const uint8_t __firmware_crc_end__;
extern const uint8_t __firmware_crc_ext_start__;
extern const uint8_t __firmware_crc_ext_end__;
}

namespace
{
constexpr uint32_t CACHE_ALIGNMENT = 32;

class ExecutableMemoryChibiOS final : public AP_HAL::ExecutableMemory
{
public:
    ExecutableMemoryChibiOS(void *allocation, void *data, uint32_t size) :
        _allocation(allocation), _data(data), _size(size) {}
    ~ExecutableMemoryChibiOS() override
    {
        chHeapFree(_allocation);
    }
    void *data() const override
    {
        return _data;
    }
    uint32_t size() const override
    {
        return _size;
    }
    uint32_t allocation_size() const override
    {
        return chHeapGetSize(_allocation) + sizeof(heap_header_t) + sizeof(*this);
    }

    bool publish(uint32_t code_size) override
    {
        if (_published || code_size == 0 || code_size > _size) {
            return false;
        }
        // Clean and invalidate relocated code in D-cache before invalidating I-cache.
        // Payload alignment/rounding ensures cache lines are exclusively ours.
        stm32_cacheBufferFlush(_data, (code_size + CACHE_ALIGNMENT - 1) & ~(CACHE_ALIGNMENT - 1));
        __DSB();
#if defined(__ICACHE_PRESENT) && __ICACHE_PRESENT == 1U
        SCB_InvalidateICache();
#endif
        __DSB();
        __ISB();
        _published = true;
        return true;
    }

private:
    void *_allocation;
    void *_data;
    uint32_t _size;
    bool _published = false;
};
}

AP_HAL::ExecutableMemory::Target ChibiOS::Util::executable_memory_target() const
{
#if defined(__ARM_PCS_VFP) && defined(__ARM_ARCH_7EM__)
#if (defined(STM32H743xx) || defined(STM32H757xx)) && __CORTEX_M == 7U
    return AP_HAL::ExecutableMemory::Target::CortexM7HardFloat;
#elif defined(STM32F405xx) && __CORTEX_M == 4U
    return AP_HAL::ExecutableMemory::Target::CortexM4HardFloat;
#else
    return AP_HAL::ExecutableMemory::Target::Unsupported;
#endif
#else
    return AP_HAL::ExecutableMemory::Target::Unsupported;
#endif
}

AP_HAL::ExecutableMemory *ChibiOS::Util::allocate_executable(uint32_t size, uint32_t alignment)
{
    // Start with explicitly verified MCU memory maps. Never fall back to
    // arbitrary fast RAM: F405 CCM cannot be used for instruction fetches.
#if defined(STM32H743xx) || defined(STM32H757xx)
    constexpr uintptr_t region_start = 0x24000000;
    constexpr uintptr_t region_end = 0x24080000;
#elif defined(STM32F405xx)
    constexpr uintptr_t region_start = 0x20000000;
    constexpr uintptr_t region_end = 0x20020000;
#else
    return nullptr;
#endif
#if defined(STM32H743xx) || defined(STM32H757xx) || defined(STM32F405xx)
    if (size == 0 || size > 16U * 1024 * 1024 ||
        (alignment != 0 && (alignment < sizeof(uintptr_t) || alignment > 4096 || (alignment & (alignment - 1)) != 0))) {
        return nullptr;
    }
    size = (size + CACHE_ALIGNMENT - 1) & ~(CACHE_ALIGNMENT - 1);
    alignment = alignment < CACHE_ALIGNMENT ? CACHE_ALIGNMENT : alignment;
    memory_heap_t *heaps;
    const memory_region *regions;
    const uint8_t count = malloc_get_heaps(&heaps, &regions);
    for (uint8_t i = 0; i < count; i++) {
        const uintptr_t address = reinterpret_cast<uintptr_t>(regions[i].address);
        if (address < region_start || address >= region_end || regions[i].size > region_end - address) {
            continue;
        }
        // Overalignment through the default heap's core provider consumes
        // padding which chHeapFree cannot recover. Own a normally aligned raw
        // block there, so alignment slack is charged and released. Fixed
        // region heaps already retain their alignment fragments in free lists.
        void *allocation = i == 0 ? chHeapAlloc(nullptr, size + alignment - 1) : chHeapAllocAligned(&heaps[i], size, alignment);
        if (allocation == nullptr) {
            continue;
        }
        const uintptr_t aligned_address = (reinterpret_cast<uintptr_t>(allocation) + alignment - 1) & ~uintptr_t(alignment - 1);
        void *data = reinterpret_cast<void *>(aligned_address);
        memset(data, 0, size);
        auto *memory = NEW_NOTHROW ExecutableMemoryChibiOS(allocation, data, size);
        if (memory == nullptr) {
            chHeapFree(allocation);
        }
        return memory;
    }
    return nullptr;
#endif
}

bool ChibiOS::Util::is_executable(const void *address, uint32_t size) const
{
    if (address == nullptr || size == 0) {
        return false;
    }
    const uintptr_t start = reinterpret_cast<uintptr_t>(address);
    if (start > UINTPTR_MAX - size) {
        return false;
    }
    const uintptr_t end = start + size;
    const auto within = [start, end](uintptr_t region_start, uintptr_t region_end) {
        return region_start < region_end && region_start <= start && end <= region_end;
    };

    // These linker ranges are the immutable memory-mapped firmware images.
    // They cover internal flash, external-XIP firmware and the mixed-flash
    // ROMFS section without claiming unused or non-mapped flash addresses.
    return within(reinterpret_cast<uintptr_t>(&__firmware_crc_start__),
                  reinterpret_cast<uintptr_t>(&__firmware_crc_end__)) ||
           within(reinterpret_cast<uintptr_t>(&__firmware_crc_ext_start__),
                  reinterpret_cast<uintptr_t>(&__firmware_crc_ext_end__));
}
#endif
