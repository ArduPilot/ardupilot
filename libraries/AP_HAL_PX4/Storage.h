#pragma once

/*
  we can optionally use flash for storage instead of FRAM. That allows
  ArduPilot to run on a wider range of boards and reduces board cost
 */
#ifndef USE_FLASH_STORAGE
#define USE_FLASH_STORAGE 0
#endif

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_PX4_Namespace.h"
#include <systemlib/perf_counter.h>
#include <AP_Common/Bitmask.h>
#include <AP_FlashStorage/AP_FlashStorage.h>

#define PX4_STORAGE_SIZE HAL_STORAGE_SIZE

#if USE_FLASH_STORAGE
// when using flash storage we use a small line size to make storage
// compact and minimise the number of erase cycles needed
#define PX4_STORAGE_LINE_SHIFT 3
#else
#define PX4_STORAGE_LINE_SHIFT 9
#endif

#define PX4_STORAGE_LINE_SIZE (1<<PX4_STORAGE_LINE_SHIFT)
#define PX4_STORAGE_NUM_LINES (PX4_STORAGE_SIZE/PX4_STORAGE_LINE_SIZE)

class PX4::PX4Storage : public AP_HAL::Storage {
public:
    PX4Storage();

    void init() {}
    void read_block(void *dst, uint16_t src, size_t n);
    void write_block(uint16_t dst, const void* src, size_t n);

    void _timer_tick(void) override;

private:
    volatile bool _initialised;
    void _storage_create(void);
    void _storage_open(void);
    void _mark_dirty(uint16_t loc, uint16_t length);
    uint8_t _buffer[PX4_STORAGE_SIZE] __attribute__((aligned(4)));
    Bitmask _dirty_mask{PX4_STORAGE_NUM_LINES};
    perf_counter_t  _perf_storage;
    perf_counter_t  _perf_errors;
    uint32_t _last_re_init_ms;

#if !USE_FLASH_STORAGE
    int _fd = -1;
    void bus_lock(bool lock);
    void _mtd_load(void);
    void _mtd_write(uint16_t line);
#if defined (CONFIG_ARCH_BOARD_PX4FMU_V4)
    irqstate_t irq_state;
#endif

#else // USE_FLASH_STORAGE
    bool _flash_write_data(uint8_t sector, uint32_t offset, const uint8_t *data, uint16_t length);
    bool _flash_read_data(uint8_t sector, uint32_t offset, uint8_t *data, uint16_t length);
    bool _flash_erase_sector(uint8_t sector);
    bool _flash_erase_ok(void);
    uint8_t _flash_page;
    bool _flash_failed;
    
    AP_FlashStorage _flash{_buffer,
            128*1024U, 
            FUNCTOR_BIND_MEMBER(&PX4Storage::_flash_write_data, bool, uint8_t, uint32_t, const uint8_t *, uint16_t),
            FUNCTOR_BIND_MEMBER(&PX4Storage::_flash_read_data, bool, uint8_t, uint32_t, uint8_t *, uint16_t),
            FUNCTOR_BIND_MEMBER(&PX4Storage::_flash_erase_sector, bool, uint8_t),
            FUNCTOR_BIND_MEMBER(&PX4Storage::_flash_erase_ok, bool)};
    
    void _flash_load(void);
    void _flash_write(uint16_t line);
#endif
};
