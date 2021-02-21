#pragma once

#include "AP_HAL_Empty.h"

class Empty::Flash : public AP_HAL::Flash {
public:
    uint32_t getpageaddr(uint32_t page) override { return 0; }
    uint32_t getpagesize(uint32_t page) override { return 0; }
    uint32_t getnumpages(void) override { return 0; }
    bool erasepage(uint32_t page) override { return false; }
    bool write(uint32_t addr, const void *buf, uint32_t count) override { return false; }
    void keep_unlocked(bool set) override {}
    bool ispageerased(uint32_t page) override { return false; }
};
