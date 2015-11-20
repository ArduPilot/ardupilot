#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include <stdlib.h>
#include <cstdio>
#include "SPIUARTDriver.h"
#include <AP_HAL/utility/RingBuffer.h>

extern const AP_HAL::HAL& hal;

#define SPIUART_DEBUG 0

#include <cassert>

#if SPIUART_DEBUG
#define debug(fmt, args ...)  do {hal.console->printf("[SPIUARTDriver]: %s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#define error(fmt, args ...)  do {fprintf(stderr,"%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
#define debug(fmt, args ...)  
#define error(fmt, args ...)  
#endif

using namespace Linux;

SPIUARTDriver::SPIUARTDriver() :
    UARTDriver(false),
    _spi(NULL),
    _spi_sem(NULL),
    _last_update_timestamp(0),
    _buffer(NULL),
    _external(false)
{
    _readbuf = NULL;
    _writebuf = NULL;
}

bool SPIUARTDriver::sem_take_nonblocking()
{
    return _spi_sem->take_nonblocking();
}

void SPIUARTDriver::sem_give()
{
    _spi_sem->give();
}

void SPIUARTDriver::begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
    if (device_path != NULL) {
        UARTDriver::begin(b,rxS,txS);
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

   if (_buffer == NULL) {
       /* Do not allocate new buffer, if we're just changing speed */
       _buffer = new uint8_t[rxS];
       if (_buffer == NULL) {
           hal.console->printf("Not enough memory\n");
           AP_HAL::panic("Not enough memory\n");
       }
   }

   _spi = hal.spi->device(AP_HAL::SPIDevice_Ublox);

   if (_spi == NULL) {
       AP_HAL::panic("Cannot get SPIDevice_Ublox");
   }

   _spi_sem = _spi->get_semaphore();

   switch (b) {
       case 4000000U:
           if (is_initialized()) {
               /* Do not allow speed changes before device is initialized, because
                * it can lead to misconfiguraration. Once the device is initialized,
                * it's sage to update speed
                */
               _spi->set_bus_speed(AP_HAL::SPIDeviceDriver::SPI_SPEED_HIGH);
               debug("Set higher SPI-frequency");
           } else {
               _spi->set_bus_speed(AP_HAL::SPIDeviceDriver::SPI_SPEED_LOW);
               debug("Set lower SPI-frequency");
           }
           break;
       default:
           _spi->set_bus_speed(AP_HAL::SPIDeviceDriver::SPI_SPEED_LOW);
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

    if (!sem_take_nonblocking()) {
        return 0;
    }

    _spi->transaction(buf, _buffer, size);

    sem_give();

    BUF_ADVANCEHEAD(_writebuf, size);

    uint16_t ret = size;

    /* Since all SPI-transactions are transfers we need update
     * the _readbuf. I do believe there is a way to encapsulate 
     * this operation since it's the same as in the
     * UARTDriver::write().  
     */

    uint8_t *buffer = _buffer;

    uint16_t _head, space;
    space = BUF_SPACE(_readbuf);

    if (space == 0) {
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


    return ret;
    
}

static const uint8_t ff_stub[300] = {0xff};
int SPIUARTDriver::_read_fd(uint8_t *buf, uint16_t n)
{
    if (_external) {
        return UARTDriver::_read_fd(buf, n);
    }

    if (!sem_take_nonblocking()) {
        return 0;
    }

    /* Make SPI transactions shorter. It can save SPI bus from keeping too
     * long. It's essential for NavIO as MPU9250 is on the same bus and 
     * doesn't like to be waiting. Making transactions more frequent but shorter
     * is a win.
     */  

    if (n > 100) {
        n = 100;
    }

    _spi->transaction(ff_stub, buf, n);

    sem_give();

    BUF_ADVANCETAIL(_readbuf, n);

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

#endif
