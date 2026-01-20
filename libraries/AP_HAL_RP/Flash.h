#pragma once

#include "AP_HAL_RP.h"

class RP::Flash : public AP_HAL::Flash {
public:
    Flash() = default;

    void init();

    uint32_t getpageaddr(uint32_t page) override;
    uint32_t getpagesize(uint32_t page) override;
    uint32_t getnumpages(void) override;
    bool erasepage(uint32_t page) override;
    bool write(uint32_t addr, const void *buf, uint32_t count) override;
    void keep_unlocked(bool set) override {};
    bool ispageerased(uint32_t page) override;

private:
    bool is_device_busy(AP_HAL::WSPIDevice* dev);

    static constexpr uint32_t RP_FLASH_SECTOR_SIZE = 4096;
    static constexpr uint32_t RP_FLASH_PAGE_SIZE = 256;
    static constexpr uint32_t STORAGE_OFFSET = (2048 * 1024 - 128 * 1024);
    static constexpr uint32_t STORAGE_NUM_PAGES = 32;  // 32 * 4KB = 128KB
};
