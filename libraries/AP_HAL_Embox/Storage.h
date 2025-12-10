#pragma once

#include "AP_HAL_Embox.h"

namespace Embox {
    class Storage : public AP_HAL::Storage {
    public:
        Storage();
        void init() override;
        void read_block(void* dst, uint16_t src, size_t n) override;
        void write_block(uint16_t dst, const void* src, size_t n) override;
    };
} // namespace Embox