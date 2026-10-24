// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <AP_HAL/ExecutableMemory.h>

#if HAL_EXECUTABLE_MEMORY_ENABLED && (CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX)
class ExecutableMemoryPosix final : public AP_HAL::ExecutableMemory
{
public:
    static ExecutableMemory *create(uint32_t size, uint32_t alignment = 0);
    ~ExecutableMemoryPosix() override;
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
        return _size + sizeof(*this);
    }
    bool publish(uint32_t code_size) override;

private:
    ExecutableMemoryPosix(void *data, uint32_t size) : _data(data), _size(size) {}
    void *_data;
    uint32_t _size;
    bool _published = false;
};
#endif
