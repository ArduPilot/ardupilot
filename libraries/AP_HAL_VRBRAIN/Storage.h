#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_VRBRAIN_Namespace.h"
#include <systemlib/perf_counter.h>

#define VRBRAIN_STORAGE_SIZE HAL_STORAGE_SIZE
#define VRBRAIN_STORAGE_MAX_WRITE 512
#define VRBRAIN_STORAGE_LINE_SHIFT 9
#define VRBRAIN_STORAGE_LINE_SIZE (1<<VRBRAIN_STORAGE_LINE_SHIFT)
#define VRBRAIN_STORAGE_NUM_LINES (VRBRAIN_STORAGE_SIZE/VRBRAIN_STORAGE_LINE_SIZE)

class VRBRAIN::VRBRAINStorage : public AP_HAL::Storage {
public:
	VRBRAINStorage();

    void init() {}
    void read_block(void *dst, uint16_t src, size_t n);
    void write_block(uint16_t dst, const void* src, size_t n);

    void _timer_tick(void);

private:
    int _fd;
    volatile bool _initialised;
    void _storage_create(void);
    void _storage_open(void);
    void _mark_dirty(uint16_t loc, uint16_t length);
    uint8_t _buffer[VRBRAIN_STORAGE_SIZE] __attribute__((aligned(4)));
    volatile uint32_t _dirty_mask;
    perf_counter_t  _perf_storage;
    perf_counter_t  _perf_errors;
    bool _have_mtd;
    void _upgrade_to_mtd(void);
    uint32_t _mtd_signature(void);
    void _mtd_write_signature(void);




    void bus_lock(bool lock);
};
