#pragma once 

#include <AP_HAL/AP_HAL.h>

#ifdef BOARD_OSD_CS_PIN

#define OSD_RX_BUF_SIZE 256
#define OSD_TX_BUF_SIZE 256

#include <AP_HAL_F4Light/AP_HAL_F4Light.h>

extern const AP_HAL::HAL& hal;

#include "osd_namespace.h"

#include "osd_core/compat.h"
#include "osd_core/Defs.h"


#define OSD_LOW_PRIORITY 115 // 15 less than main task so runs almost only in delay() time - 1/16 of main thread
#define OSD_HIGH_PRIORITY 99 //  1 more than main so uses 2/3 of CPU

namespace OSDns {// OSD interface emulates UART

    void osd_begin(AP_HAL::OwnPtr<F4Light::SPIDevice> spi);
    void osd_loop();

    int16_t osd_available();

    int16_t osd_getc();
    void osd_dequeue();

    uint32_t osd_txspace();
    void osd_putc(uint8_t c); 

    void max_do_transfer(const uint8_t *buffer, uint16_t len);
    void update_max_buffer(const uint8_t *buffer, uint16_t len);

    inline uint32_t millis(){ return AP_HAL::millis(); }
}
#endif
