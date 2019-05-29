/*
  interface to flash read/write
 */
#pragma once

#include <stdint.h>
#include "AP_HAL_Namespace.h"

class AP_HAL::Flash {
public:
    virtual uint32_t getpageaddr(uint32_t page) = 0;
    virtual uint32_t getpagesize(uint32_t page) = 0;
    virtual uint32_t getnumpages(void) = 0;
    virtual bool erasepage(uint32_t page) = 0;
    virtual bool write(uint32_t addr, const void *buf, uint32_t count) = 0;
    virtual void keep_unlocked(bool set) = 0;
    virtual bool ispageerased(uint32_t page) = 0;
};
