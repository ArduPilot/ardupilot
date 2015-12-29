/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_QURT
#include <stdlib.h>
#include <unistd.h>
#include "UARTDriver.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <fcntl.h>
#include <dev_fs_lib_serial.h>
#include <AP_HAL/utility/RingBuffer.h>

using namespace QURT;

extern const AP_HAL::HAL& hal;

UARTDriver::UARTDriver(const char *name) :
    devname(name)
{
}

void UARTDriver::begin(uint32_t b) 
{
    begin(b, 16384, 16384);
}

static const struct {
    uint32_t baudrate;
    enum DSPAL_SERIAL_BITRATES arg;
} baudrate_table[] = {
    {    9600, DSPAL_SIO_BITRATE_9600 },
    {   14400, DSPAL_SIO_BITRATE_14400 },
    {   19200, DSPAL_SIO_BITRATE_19200 },
    {   38400, DSPAL_SIO_BITRATE_38400 },
    {   57600, DSPAL_SIO_BITRATE_57600 },
    {   76800, DSPAL_SIO_BITRATE_76800 },
    {  115200, DSPAL_SIO_BITRATE_115200 },
    {  230400, DSPAL_SIO_BITRATE_230400 },
    {  250000, DSPAL_SIO_BITRATE_250000 },
    {  460800, DSPAL_SIO_BITRATE_460800 },
    {  921600, DSPAL_SIO_BITRATE_921600 },
    { 2000000, DSPAL_SIO_BITRATE_2000000 },
};

extern "C" {
static void read_callback_trampoline(void *, char *, size_t );
}

static void read_callback_trampoline(void *ctx, char *buf, size_t size)
{
    ((UARTDriver *)ctx)->_read_callback(buf, size);
}

/*
  callback for incoming data
 */
void UARTDriver::_read_callback(char *buf, size_t size)
{
    if (readbuf == nullptr) {
        return;
    }
    uint32_t n = readbuf->write((const uint8_t *)buf, size);
    if (n != size) {
        HAP_PRINTF("read_callback lost %u %u", n, size);
    }
}

void UARTDriver::begin(uint32_t b, uint16_t rxS, uint16_t txS) 
{
    if (devname == nullptr) {
        return;
    }
    if (rxS < 4096) {
        rxS = 4096;
    }
    if (txS < 4096) {
        txS = 4096;
    }
    if (fd == -1) {
        fd = open(devname, O_RDWR | O_NONBLOCK);
        if (fd == -1) {
            AP_HAL::panic("Unable to open %s", devname);
        }
    }

    /*
      allocate the read buffer
      we allocate buffers before we successfully open the device as we
      want to allocate in the early stages of boot, and cause minimum
      thrashing of the heap once we are up. The ttyACM0 driver may not
      connect for some time after boot
     */
    if (rxS != 0 && (readbuf == nullptr || rxS != readbuf->get_size())) {
        initialised = false;
        if (readbuf != nullptr) {
            delete readbuf;
        }
        readbuf = new ByteBuffer(rxS);
    }

    /*
      allocate the write buffer
    */
    if (txS != 0 && (writebuf == nullptr || txS != writebuf->get_size())) {
        initialised = false;
        if (writebuf != nullptr) {
            delete writebuf;
        }
        writebuf = new ByteBuffer(txS);
    }
    
    struct dspal_serial_ioctl_receive_data_callback callback;
    callback.context = this;
    callback.rx_data_callback_func_ptr = read_callback_trampoline;
    int ret = ioctl(fd, SERIAL_IOCTL_SET_RECEIVE_DATA_CALLBACK, (void *)&callback);

    if (b != 0) {
        for (uint8_t i=0; i<ARRAY_SIZE(baudrate_table); i++) {
            if (b <= baudrate_table[i].baudrate) {
                struct dspal_serial_ioctl_data_rate rate;
                rate.bit_rate = baudrate_table[i].arg;
                ret = ioctl(fd, SERIAL_IOCTL_SET_DATA_RATE, (void *)&rate);
                break;
            }
        }
    }

    if (readbuf && writebuf && fd != -1) {
        initialised = true;
    }
}

void UARTDriver::end() 
{
    initialised = false;
    if (fd != -1) {
        ::close(fd);
        fd = -1;
    }
    if (readbuf) {
        delete readbuf;
        readbuf = nullptr;
    }
    if (writebuf) {
        delete writebuf;
        writebuf = nullptr;
    }

}

void UARTDriver::flush() 
{
}

bool UARTDriver::is_initialized() 
{ 
    return fd != -1 && initialised; 
}

void UARTDriver::set_blocking_writes(bool blocking) 
{
    nonblocking_writes = !blocking;
}

bool UARTDriver::tx_pending()
{
    return false;
}

/* QURT implementations of Stream virtual methods */
int16_t UARTDriver::available() 
{ 
    if (!initialised) {
        return 0;
    }
    return readbuf->available();
}

int16_t UARTDriver::txspace() 
{ 
    if (!initialised) {
        return 0;
    }
    return writebuf->space();
}

int16_t UARTDriver::read() 
{ 
    uint8_t c;
    if (!initialised) {
        return -1;
    }
    if (!lock.take(0)) {
        return 0;
    }
    if (readbuf->empty()) {
        lock.give();
        return -1;
    }
    readbuf->read(&c, 1);
    lock.give();
    return c;
}

/* QURT implementations of Print virtual methods */
size_t UARTDriver::write(uint8_t c) 
{
    if (!initialised) {
        return 0;
    }
    if (!lock.take(0)) {
        return 0;
    }

    while (writebuf->space() == 0) {
        if (nonblocking_writes) {
            lock.give();
            return 0;
        }
        hal.scheduler->delay_microseconds(1000);
    }
    writebuf->write(&c, 1);
    lock.give();
    return 1;
}

size_t UARTDriver::write(const uint8_t *buffer, size_t size)
{
    if (!initialised) {
        return 0;
    }
    if (!nonblocking_writes) {
        /*
          use the per-byte delay loop in write() above for blocking writes
         */
        size_t ret = 0;
        while (size--) {
            if (write(*buffer++) != 1) break;
            ret++;
        }
        return ret;
    }

    if (!lock.take(0)) {
        return 0;
    }
    size_t ret = writebuf->write(buffer, size);
    lock.give();
    return ret;
}

/*
  push any pending bytes to/from the serial port. This is called at
  1kHz in the timer thread
 */
void UARTDriver::timer_tick(void)
{
    uint16_t n;

    if (fd == -1 || !initialised || !lock.take_nonblocking()) {
        return;
    }

    in_timer = true;
    
    // write any pending bytes
    n = writebuf->available();
    if (n == 0) {
        in_timer = false;
        lock.give();
        return;
    }
    if (n > 64) {
        n = 64;
    }
    uint8_t buf[n];
    writebuf->read(buf, n);
    ::write(fd, buf, n);
    lock.give();
    
    in_timer = false;
}

#endif
