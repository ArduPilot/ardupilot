#include "UARTDriver.h"

#include <stdio.h>
#include <errno.h>
#include <termios.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

using namespace Linux;

LinuxUARTDriver::LinuxUARTDriver() :
    device_path(NULL),
    _fd(-1)
{
}

/*
  set the tty device to use for this UART
 */
void LinuxUARTDriver::set_device_path(const char *path)
{
    device_path = path;
}

/*
  open the tty
 */
void LinuxUARTDriver::begin(uint32_t b) 
{
    if (device_path == NULL) {
        return;
    }

    if (_fd == -1) {
        _fd = open(device_path, O_RDWR);
        if (_fd == -1) {
            ::printf("UARTDriver: Failed to open %s - %s\n", 
                     device_path,
                     strerror(errno));
            return;
        }
    }

    /* if baudrate has been specified, then set the baudrate */
    if (b != 0) {
        struct termios t;
        tcgetattr(_fd, &t);
        cfsetspeed(&t, b);

        // disable LF -> CR/LF
        t.c_oflag &= ~ONLCR;
        tcsetattr(_fd, TCSANOW, &t);
    }
}

void LinuxUARTDriver::begin(uint32_t b, uint16_t rxS, uint16_t txS) 
{
    // ignore buffer sizes for now
    begin(b);
}

/*
  shutdown a UART
 */
void LinuxUARTDriver::end() 
{
    if (_fd != -1) {
        close(_fd);
        _fd = -1;
    }
}


void LinuxUARTDriver::flush() 
{
    // we are not doing any buffering, so flush is a no-op
}


/*
  return true if the UART is initialised
 */
bool LinuxUARTDriver::is_initialized() 
{
    return (_fd != -1);
}


/*
  enable or disable blocking writes
 */
void LinuxUARTDriver::set_blocking_writes(bool blocking) 
{
    unsigned v;
    if (_fd == -1) {
        return;
    }

    v = fcntl(_fd, F_GETFL, 0);
    
    if (blocking) {
        v &= ~O_NONBLOCK;
    } else {
        v |= O_NONBLOCK;
    }

    fcntl(_fd, F_SETFL, v);    
}

/*
  do we have any bytes pending transmission?
 */
bool LinuxUARTDriver::tx_pending() 
{ 
    // no buffering, so always false
    return false; 
}


/*
  return the number of bytes available to be read
 */
int16_t LinuxUARTDriver::available() 
{ 
    int nread;
    if (_fd == -1) {
        return 0;
    }

    nread = 0;
    if (ioctl(_fd, FIONREAD, (unsigned long)&nread) == 0) {
        return nread;
    }
    // ioctl failed??
    return 0;
}

/*
  how many bytes are available in the output buffer?
 */
int16_t LinuxUARTDriver::txspace() 
{ 
    // for now lie and say we always have 128, we will need a ring
    // buffer later and a IO thread
    return 128; 
}

int16_t LinuxUARTDriver::read() 
{ 
    char c;
    if (_fd == -1) {
        return -1;
    }
    if (::read(_fd, &c, 1) == 1) {
        return (int16_t)c;
    }
    return -1;
}

/* Linux implementations of Print virtual methods */
size_t LinuxUARTDriver::write(uint8_t c) 
{ 
    if (_fd == -1) {
        return 0;
    }
    if (::write(_fd, &c, 1) == 1) {
        return 1;
    }
    return 0;
}
