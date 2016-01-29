#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_QURT_Namespace.h"
#include "Semaphores.h"
#include <stdio.h>

#define QURT_STORAGE_SIZE HAL_STORAGE_SIZE

class QURT::Storage : public AP_HAL::Storage
{
public:
    Storage() {}

    static Storage *from(AP_HAL::Storage *storage) {
        return static_cast<Storage*>(storage);
    }

    void init() {}
    void read_block(void *dst, uint16_t src, size_t n);
    void write_block(uint16_t dst, const void* src, size_t n);

    static volatile bool dirty;
    static uint8_t buffer[QURT_STORAGE_SIZE];
    static Semaphore lock;
};

