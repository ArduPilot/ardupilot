#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include <stdlib.h>
#include <cstdio>
#include "SPIUARTDriver.h"
#include "../AP_HAL/utility/RingBuffer.h"

extern const AP_HAL::HAL& hal;

#define SPIUART_DEBUG 1
#if SPIUART_DEBUG
#include <cassert>
#define debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#define error(fmt, args ...)  do {fprintf(stderr,"%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
#define debug(fmt, args ...)  
#define error(fmt, args ...)  
#endif

using namespace Linux;

LinuxSPIUARTDriver::LinuxSPIUARTDriver() :
    LinuxUARTDriver(false),
    _spi(NULL),
    _spi_sem(NULL),
    _last_update_timestamp(0),
    _buffer(NULL),
    _external(false)
{
    _readbuf = NULL;
    _writebuf = NULL;
}

bool LinuxSPIUARTDriver::sem_take_nonblocking()
{
    return _spi_sem->take_nonblocking();
}

void LinuxSPIUARTDriver::sem_give()
{
    _spi_sem->give();
}

void LinuxSPIUARTDriver::begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
    if (device_path != NULL) {
        LinuxUARTDriver::begin(b,rxS,txS);
        if ( is_initialized()) {
            _external = true;
            return;
        }
    }

   if (rxS < 1024) {
       rxS = 2048;
   }
   if (txS < 1024) {
       txS = 2048;
   }

   /*
     allocate the read buffer
   */
   if (rxS != 0 && rxS != _readbuf_size) {
       _readbuf_size = rxS;
       if (_readbuf != NULL) {
           free(_readbuf);
       }
       _readbuf = (uint8_t *)malloc(_readbuf_size);
       _readbuf_head = 0;
       _readbuf_tail = 0;
   }

   /*
     allocate the write buffer
   */
   if (txS != 0 && txS != _writebuf_size) {
       _writebuf_size = txS;
       if (_writebuf != NULL) {
           free(_writebuf);
       }
       _writebuf = (uint8_t *)malloc(_writebuf_size);
       _writebuf_head = 0;
       _writebuf_tail = 0;
   }

   _buffer = new uint8_t[rxS];

   _spi = hal.spi->device(AP_HAL::SPIDevice_Ublox);

   if (_spi == NULL) {
       hal.scheduler->panic("Cannot get SPIDevice_MPU9250");
   }

   _spi_sem = _spi->get_semaphore();

   _spi->set_bus_speed(AP_HAL::SPIDeviceDriver::SPI_SPEED_HIGH);
   _initialised = true;
}

int LinuxSPIUARTDriver::_write_fd(const uint8_t *buf, uint16_t size)
{
    if (_external) {
        return LinuxUARTDriver::_write_fd(buf,size);
    } 

    if (!sem_take_nonblocking()) {
        return 0;
    }

    _spi->transaction(buf, _buffer, size);

    BUF_ADVANCEHEAD(_writebuf, size);

    uint16_t ret = size;

    /* Since all SPI-transactions are transfers we need update
     * the _readbuf. I do believe there is a way to encapsulate 
     * this operation since it's the same as in the
     * LinuxUARTDriver::write().  
     */

    uint8_t *buffer = _buffer;

    uint16_t _head, space;
    space = BUF_SPACE(_readbuf);

    if (space == 0) {
        sem_give();
        return ret;
    }

    if (size > space) {
        size = space;
    }

    if (_readbuf_tail < _head) {
        // perform as single memcpy
        assert(_readbuf_tail+size <= _readbuf_size);
        memcpy(&_readbuf[_readbuf_tail], buffer, size);
        BUF_ADVANCETAIL(_readbuf, size);
        sem_give();
        return ret;
    }

    // perform as two memcpy calls
    uint16_t n = _readbuf_size - _readbuf_tail;
    if (n > size) n = size;
    assert(_readbuf_tail+n <= _readbuf_size);
    memcpy(&_readbuf[_readbuf_tail], buffer, n);
    BUF_ADVANCETAIL(_readbuf, n);
    buffer += n;
    n = size - n;
    if (n > 0) {
        assert(_readbuf_tail+n <= _readbuf_size);
        memcpy(&_readbuf[_readbuf_tail], buffer, n);
        BUF_ADVANCETAIL(_readbuf, n);
    }

    sem_give();

    return ret;
    
}

static const uint8_t ff_stub[3000] = {0xff};
int LinuxSPIUARTDriver::_read_fd(uint8_t *buf, uint16_t n)
{
    if (_external) {
        return LinuxUARTDriver::_read_fd(buf, n);
    }

    if (!sem_take_nonblocking()) {
        return 0;
    }

    _spi->transaction(ff_stub, buf, n);

    BUF_ADVANCETAIL(_readbuf, n);

    sem_give();
    return n;
}

void LinuxSPIUARTDriver::_timer_tick(void)
{
    if (_external) {
        LinuxUARTDriver::_timer_tick();
        return;
    }
    /* lower the update rate */
    if (hal.scheduler->micros() - _last_update_timestamp < 50000) {
        return;
    }

    LinuxUARTDriver::_timer_tick();

    _last_update_timestamp = hal.scheduler->micros();
}

#endif
