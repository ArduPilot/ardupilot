#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_ChibiOS_Namespace.h"
#include "Semaphores.h"
#include "hwdef/common/flash.h"

class ChibiOS::Flash : public AP_HAL::Flash {
public:
    uint32_t getpageaddr(uint32_t page) override { return stm32_flash_getpageaddr(page); }
    uint32_t getpagesize(uint32_t page) override { return stm32_flash_getpagesize(page); }
    uint32_t getnumpages(void) override { return stm32_flash_getnumpages(); }
    bool erasepage(uint32_t page) override {
        sem.take(HAL_SEMAPHORE_BLOCK_FOREVER);
        bool ret = stm32_flash_erasepage(page);
        sem.give();
        return ret;
    }
    bool write(uint32_t addr, const void *buf, uint32_t count) override {
        sem.take(HAL_SEMAPHORE_BLOCK_FOREVER);
        bool ret = (stm32_flash_write(addr, buf, count) == count);
        sem.give();
        return ret;
    }
    void keep_unlocked(bool set) override { stm32_flash_keep_unlocked(set); }
    bool ispageerased(uint32_t page) override { return stm32_flash_ispageerased(page); }

private:
    Semaphore sem;
};
