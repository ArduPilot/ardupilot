#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "RPIOUARTDriver.h"

#include <stdlib.h>
#include <cstdio>

#include <AP_HAL/utility/RingBuffer.h>

#include "px4io_protocol.h"

#define RPIOUART_POLL_TIME_INTERVAL 10000

extern const AP_HAL::HAL& hal;

#define RPIOUART_DEBUG 0

#include <cassert>

#if RPIOUART_DEBUG
#define debug(fmt, args ...)  do {hal.console->printf("[RPIOUARTDriver]: %s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#define error(fmt, args ...)  do {fprintf(stderr,"%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
#define debug(fmt, args ...)  
#define error(fmt, args ...)  
#endif

using namespace Linux;

RPIOUARTDriver::RPIOUARTDriver() :
    UARTDriver(false),
    _spi(NULL),
    _spi_sem(NULL),
    _last_update_timestamp(0),
    _external(false),
    _need_set_baud(false),
    _baudrate(0)
{
    _readbuf = NULL;
    _writebuf = NULL;
}

bool RPIOUARTDriver::sem_take_nonblocking()
{
    return _spi_sem->take_nonblocking();
}

void RPIOUARTDriver::sem_give()
{
    _spi_sem->give();
}

bool RPIOUARTDriver::isExternal()
{
    return _external;
}

void RPIOUARTDriver::begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
    //hal.console->printf("[RPIOUARTDriver]: begin \n");
    
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
    
    _initialised = false;
    while (_in_timer) hal.scheduler->delay(1);

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

   _spi = hal.spi->device(AP_HAL::SPIDevice_RASPIO);

   if (_spi == NULL) {
       AP_HAL::panic("Cannot get SPIDevice_RASPIO");
   }

   _spi_sem = _spi->get_semaphore();
    
    if (_spi_sem == NULL) {
        AP_HAL::panic("PANIC: RASPIOUARTDriver did not get "
                                  "valid SPI semaphore!");
        return; // never reached
    }
    
    /* set baudrate */
    _baudrate = b;
    _need_set_baud = true;
    while (_need_set_baud) {
        hal.scheduler->delay(1);
    }

    if (_writebuf_size != 0 && _readbuf_size != 0) {
        _initialised = true;
    }

}

int RPIOUARTDriver::_write_fd(const uint8_t *buf, uint16_t n)
{
    if (_external) {
        return UARTDriver::_write_fd(buf, n);
    } 

    return -1;
}

int RPIOUARTDriver::_read_fd(uint8_t *buf, uint16_t n)
{
    if (_external) {
        return UARTDriver::_read_fd(buf, n);
    }
    
    return -1;
}

void RPIOUARTDriver::_timer_tick(void)
{
    if (_external) {
        UARTDriver::_timer_tick();
        return;
    }
    
    /* set the baudrate of raspilotio */
    if (_need_set_baud) {
        
        if (_baudrate != 0) {
            
            if (!_spi_sem->take_nonblocking()) {
                return;
            }
            
            struct IOPacket _dma_packet_tx, _dma_packet_rx;
            
            _dma_packet_tx.count_code = 2 | PKT_CODE_WRITE;
            _dma_packet_tx.page = PX4IO_PAGE_UART_BUFFER;
            _dma_packet_tx.offset = 0;
            _dma_packet_tx.regs[0] = _baudrate & 0xffff;
            _dma_packet_tx.regs[1] = _baudrate >> 16;
            _dma_packet_tx.crc = 0;
            _dma_packet_tx.crc = crc_packet(&_dma_packet_tx);
            
            _spi->transaction((uint8_t *)&_dma_packet_tx, (uint8_t *)&_dma_packet_rx, sizeof(_dma_packet_tx));
            
            hal.scheduler->delay(1);
            
            _spi_sem->give();
            
        }
        
        _need_set_baud = false;
    }
    /* finish set */
    
    if (!_initialised) return;
    
    /* lower the update rate */
    if (AP_HAL::micros() - _last_update_timestamp < RPIOUART_POLL_TIME_INTERVAL) {
        return;
    }
    
    _in_timer = true;
    
    if (!_spi_sem->take_nonblocking()) {
        return;
    }
    
    struct IOPacket _dma_packet_tx, _dma_packet_rx;
    
    /* get write_buf bytes */
    uint16_t _tail;
    uint16_t n = BUF_AVAILABLE(_writebuf);
    
    if (n > PKT_MAX_REGS * 2) {
        n = PKT_MAX_REGS * 2;
    }
    
    uint16_t _max_size = _baudrate / 10 / (1000000 / RPIOUART_POLL_TIME_INTERVAL);
    if (n > _max_size) {
        n = _max_size;
    }
    
    if (n > 0) {
        uint16_t n1 = _writebuf_size - _writebuf_head;
        if (n1 >= n) {
            // do as a single write
            memcpy( &((uint8_t *)_dma_packet_tx.regs)[0], &_writebuf[_writebuf_head], n );
        } else {
            // split into two writes
            memcpy( &((uint8_t *)_dma_packet_tx.regs)[0], &_writebuf[_writebuf_head], n1 );
            memcpy( &((uint8_t *)_dma_packet_tx.regs)[n1], &_writebuf[0], n-n1 );
        }
        
        BUF_ADVANCEHEAD(_writebuf, n);
    }
    
    _dma_packet_tx.count_code = PKT_MAX_REGS | PKT_CODE_SPIUART;
    _dma_packet_tx.page = PX4IO_PAGE_UART_BUFFER;
    _dma_packet_tx.offset = n;
    /* end get write_buf bytes */
    
    _dma_packet_tx.crc = 0;
    _dma_packet_tx.crc = crc_packet(&_dma_packet_tx);
    /* set raspilotio to read uart data */
    _spi->transaction((uint8_t *)&_dma_packet_tx, (uint8_t *)&_dma_packet_rx, sizeof(_dma_packet_tx));
    
    hal.scheduler->delay_microseconds(100);
    
    /* get uart data from raspilotio */
    _dma_packet_tx.count_code = 0 | PKT_CODE_READ;
    _dma_packet_tx.page = 0;
    _dma_packet_tx.offset = 0;
    memset( &_dma_packet_tx.regs[0], 0, PKT_MAX_REGS*sizeof(uint16_t) );
    _dma_packet_tx.crc = 0;
    _dma_packet_tx.crc = crc_packet(&_dma_packet_tx);
    _spi->transaction((uint8_t *)&_dma_packet_tx, (uint8_t *)&_dma_packet_rx, sizeof(_dma_packet_tx));
    
    hal.scheduler->delay_microseconds(100);
    
    /* release sem */
    _spi_sem->give();
    
    /* add bytes to read buf */
    uint16_t _head;
    n = BUF_SPACE(_readbuf);
    
    if (_dma_packet_rx.page == PX4IO_PAGE_UART_BUFFER) {
        
        if (n > _dma_packet_rx.offset) {
            n = _dma_packet_rx.offset;
        }
        
        if (n > PKT_MAX_REGS * 2) {
            n = PKT_MAX_REGS * 2;
        }
        
        if (n > 0) {
            uint16_t n1 = _readbuf_size - _readbuf_tail;
            if (n1 >= n) {
                // one read will do
                memcpy( &_readbuf[_readbuf_tail], &((uint8_t *)_dma_packet_rx.regs)[0], n );
            } else {
                memcpy( &_readbuf[_readbuf_tail], &((uint8_t *)_dma_packet_rx.regs)[0], n1 );
                memcpy( &_readbuf[0], &((uint8_t *)_dma_packet_rx.regs)[n1], n-n1 );
            }
            
            BUF_ADVANCETAIL(_readbuf, n);
        }
        
    }
    
    _in_timer = false;
    
    _last_update_timestamp = AP_HAL::micros();
}

#endif
