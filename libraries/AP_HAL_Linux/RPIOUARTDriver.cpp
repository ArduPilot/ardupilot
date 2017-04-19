#include "RPIOUARTDriver.h"

#include <stdlib.h>
#include <cstdio>

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include "px4io_protocol.h"

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
    _dev(nullptr),
    _external(false),
    _need_set_baud(false),
    _baudrate(0)
{
}

bool RPIOUARTDriver::isExternal()
{
    return _external;
}

void RPIOUARTDriver::begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
    //hal.console->printf("[RPIOUARTDriver]: begin \n");

    if (device_path != nullptr) {
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

    _readbuf.set_size(rxS);
    _writebuf.set_size(txS);

    if (!_registered_callback) {
        _dev = hal.spi->get_device("raspio");
        _registered_callback = true;
        _dev->register_periodic_callback(100000, FUNCTOR_BIND_MEMBER(&RPIOUARTDriver::_bus_timer, void));
    }

    /* set baudrate */
    _baudrate = b;
    _need_set_baud = true;

    if (_writebuf.get_size() && _readbuf.get_size()) {
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
}

void RPIOUARTDriver::_bus_timer(void)
{
    /* set the baudrate of raspilotio */
    if (_need_set_baud) {
        if (_baudrate != 0) {
            struct IOPacket _packet_tx = {0}, _packet_rx = {0};

            _packet_tx.count_code = 2 | PKT_CODE_WRITE;
            _packet_tx.page = PX4IO_PAGE_UART_BUFFER;
            _packet_tx.regs[0] = _baudrate & 0xffff;
            _packet_tx.regs[1] = _baudrate >> 16;
            _packet_tx.crc = crc_packet(&_packet_tx);

            _dev->transfer((uint8_t *)&_packet_tx, sizeof(_packet_tx),
                           (uint8_t *)&_packet_rx, sizeof(_packet_rx));

            hal.scheduler->delay(1);
        }

        _need_set_baud = false;
    }
    /* finish set */

    if (!_initialised) {
        return;
    }

    _in_timer = true;

    struct IOPacket _packet_tx = {0}, _packet_rx = {0};

    /* get write_buf bytes */
    uint32_t max_size = MIN((uint32_t) PKT_MAX_REGS * 2,
                            _baudrate / 10 / (1000000 / 100000));
    uint32_t n = MIN(_writebuf.available(), max_size);

    _writebuf.read(&((uint8_t *)_packet_tx.regs)[0], n);
    _packet_tx.count_code = PKT_MAX_REGS | PKT_CODE_SPIUART;
    _packet_tx.page = PX4IO_PAGE_UART_BUFFER;
    _packet_tx.offset = n;
    _packet_tx.crc = crc_packet(&_packet_tx);
    /* end get write_buf bytes */

    /* set raspilotio to read uart data */
    _dev->transfer((uint8_t *)&_packet_tx, sizeof(_packet_tx),
                   (uint8_t *)&_packet_rx, sizeof(_packet_rx));

    hal.scheduler->delay_microseconds(100);

    /* get uart data from raspilotio */
    memset(&_packet_tx, 0, sizeof(_packet_tx));
    _packet_tx.count_code = PKT_CODE_READ;
    _packet_tx.crc = crc_packet(&_packet_tx);
    _dev->transfer((uint8_t *)&_packet_tx, sizeof(_packet_tx),
                   (uint8_t *)&_packet_rx, sizeof(_packet_rx));

    hal.scheduler->delay_microseconds(100);

    if (_packet_rx.page == PX4IO_PAGE_UART_BUFFER) {
        /* add bytes to read buf */
        max_size = MIN(_packet_rx.offset, PKT_MAX_REGS * 2);
        n = MIN(_readbuf.space(), max_size);

        _readbuf.write(&((uint8_t *)_packet_rx.regs)[0], n);
    }

    _in_timer = false;
}
