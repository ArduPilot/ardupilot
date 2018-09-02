/*
  (c) 2017 night_ghost@ykoctpa.ru
  
 * UART_OSD.cpp --- AP_HAL_F4Light OSD implementation via fake UART
 *
 */

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_F4LIGHT && defined(BOARD_OSD_NAME) && defined(BOARD_OSD_CS_PIN)

#include "UART_OSD.h"

#include "SPIDevice.h"

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>

#include <gpio_hal.h>

#include "osd/osd.h"

using namespace F4Light;
using namespace OSDns;

UART_OSD::UART_OSD():
    _initialized(false),
    _blocking(true)
{
}

void UART_OSD::begin(uint32_t baud) {
    (void)baud;
    
    if(_initialized) return;

    OSDns::osd_begin(F4Light::SPIDeviceManager::_get_device(BOARD_OSD_NAME));
    _initialized = true;
}


uint32_t UART_OSD::available() {
    return OSDns::osd_available(); 
}

int16_t UART_OSD::read() {
    if (available() <= 0) return -1;
    return OSDns::osd_getc();
}

uint32_t UART_OSD::txspace() {
    return OSDns::osd_txspace();
}

size_t UART_OSD::write(uint8_t c) {

    if (!_initialized) { 
        return 0;
    }

    OSDns::osd_putc(c);
     
    return 1;
}

size_t UART_OSD::write(const uint8_t *buffer, size_t size)
{
    size_t n = 0;
    while (size--) {
        n += write(*buffer++);
    }
    return n;
}

#endif // CONFIG_HAL_BOARD
