#include "SPIUARTDriver.h"

#include <assert.h>
#include <stdlib.h>
#include <cstdio>

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL &hal;

#ifdef SPIUART_DEBUG
#include <stdio.h>
#define debug(fmt, args ...)  do {hal.console->printf("[SPIUARTDriver]: %s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#define error(fmt, args ...)  do {fprintf(stderr,"%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
#define debug(fmt, args ...)
#define error(fmt, args ...)
#endif

using namespace Linux;

SPIUARTDriver::SPIUARTDriver()
    : UARTDriver(false)
{
}

void SPIUARTDriver::begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
    if (device_path != nullptr) {
        UARTDriver::begin(b, rxS, txS);
        if (is_initialized()) {
            _external = true;
            return;
        }
    }

    if (!is_initialized()) {
        _dev = hal.spi->get_device("ublox");
        if (!_dev) {
            return;
        }
    }

    if (rxS < 1024) {
        rxS = 2048;
    }
    if (txS < 1024) {
        txS = 2048;
    }

    _readbuf.set_size(rxS);
    _writebuf.set_size(txS);

    if (_buffer == nullptr) {
        /* Do not allocate new buffer, if we're just changing speed */
        _buffer = new uint8_t[rxS];
        if (_buffer == nullptr) {
            hal.console->printf("Not enough memory\n");
            AP_HAL::panic("Not enough memory\n");
        }
    }

    switch (b) {
    case 4000000U:
        if (is_initialized()) {
            /* Do not allow speed changes before device is initialized, because
                * it can lead to misconfiguraration. Once the device is initialized,
                * it's sage to update speed
                */
            _dev->set_speed(AP_HAL::Device::SPEED_HIGH);
            debug("Set higher SPI-frequency");
        } else {
            _dev->set_speed(AP_HAL::Device::SPEED_LOW);
            debug("Set lower SPI-frequency");
        }
        break;
    default:
        _dev->set_speed(AP_HAL::Device::SPEED_LOW);
        debug("Set lower SPI-frequency");
        debug("%s: wrong baudrate (%u) for SPI-driven device. setting default speed", __func__, b);
        break;
    }

    _initialised = true;
}

int SPIUARTDriver::_write_fd(const uint8_t *buf, uint16_t size)
{
    if (_external) {
        return UARTDriver::_write_fd(buf,size);
    }

    if (!_dev->get_semaphore()->take_nonblocking()) {
        return 0;
    }

    _dev->transfer_fullduplex(buf, _buffer, size);

    _dev->get_semaphore()->give();

    uint16_t ret = size;

    /* Since all SPI-transactions are transfers we need update
     * the _readbuf. I do believe there is a way to encapsulate
     * this operation since it's the same as in the
     * UARTDriver::write().
     */
    _readbuf.write(_buffer, size);

    return ret;
}

int SPIUARTDriver::_read_fd(uint8_t *buf, uint16_t n)
{
    static uint8_t ff_stub[100] = {0xff};

    if (_external) {
        return UARTDriver::_read_fd(buf, n);
    }

    /* Make SPI transactions shorter. It can save SPI bus from keeping too
     * long. It's essential for NavIO as MPU9250 is on the same bus and
     * doesn't like to be waiting. Making transactions more frequent but shorter
     * is a win.
     */
    n = MIN(n, 100);

    if (!_dev->get_semaphore()->take_nonblocking()) {
        return 0;
    }

    _dev->transfer_fullduplex(ff_stub, buf, n);
    _dev->get_semaphore()->give();

    return n;
}

void SPIUARTDriver::_timer_tick(void)
{
    if (_external) {
        UARTDriver::_timer_tick();
        return;
    }

    /* lower the update rate */
    if (AP_HAL::micros() - _last_update_timestamp < 10000) {
        return;
    }

    UARTDriver::_timer_tick();

    _last_update_timestamp = AP_HAL::micros();
}
