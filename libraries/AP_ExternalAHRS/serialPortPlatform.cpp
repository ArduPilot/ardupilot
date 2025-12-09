/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_INERTIALSENSE_ENABLED

#include "serialPortPlatform.h"

#include "AP_HAL/AP_HAL_Namespace.h"
#include "AP_HAL/HAL.h"
#include "base_port.h"
#include "serialPort.h"
#include "ISConstants.h"

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

typedef struct
{
    AP_HAL::UARTDriver *uart;
} serialPortHandle;

static int serialPortSleepPlatform(int sleepMilliseconds);
static int serialPortFlushPlatform(port_handle_t port);
static int serialPortDrainPlatform(port_handle_t port);
static int serialPortReadTimeoutPlatform(port_handle_t port, unsigned char* buffer, unsigned int readCount, int timeoutMilliseconds);

// Return 1 on success, 0 on failure
static int serialPortOpenPlatform(port_handle_t port, void *_uart)
{
    auto uart = (AP_HAL::UARTDriver*)_uart;

    serial_port_t* serialPort = (serial_port_t*)port;
    if (serialPort->handle != 0)
        return 1;

    char portName[16];
    snprintf(portName, 16, "uart");

    serialPortSetPort(port, portName);
    serialPort->baudRate = uart->get_baud_rate();
    serialPortHandle* handle = (serialPortHandle*)calloc(1, sizeof(serialPortHandle));
    serialPort->handle = handle;
    ((serialPortHandle*)serialPort->handle)->uart = uart;

    return 1;    // success
}

static int serialPortIsOpenPlatform(port_handle_t port)
{
    serial_port_t* serialPort = (serial_port_t*)port;
    serialPortHandle* handle = (serialPortHandle*)serialPort->handle;
    if (handle == 0)
        return 0;

    return ((serialPortHandle*)serialPort->handle)->uart->is_initialized();
}

static int serialPortClosePlatform(port_handle_t port)
{
    serial_port_t* serialPort = (serial_port_t*)port;
    serialPortHandle* handle = (serialPortHandle*)serialPort->handle;
    if (handle == 0)
        return 0;

    // When closing, let's flush any pending data, as this could potentially hang the close()
    serialPortFlushPlatform(port);

    free(serialPort->handle);
    serialPort->handle = 0;

    return 1;
}

static int serialPortFlushPlatform(port_handle_t port)
{
    serial_port_t* serialPort = (serial_port_t*)port;
    serialPortHandle* handle = (serialPortHandle*)serialPort->handle;
    if (handle == 0)
        return 0;

    handle->uart->flush();

    return 1;
}

static int serialPortDrainPlatform(port_handle_t port)
{
    serial_port_t* serialPort = (serial_port_t*)port;
    serialPortHandle* handle = (serialPortHandle*)serialPort->handle;
    if (handle == 0)
        return 0;

    handle->uart->flush();

    return 1;
}

static int serialPortReadTimeoutPlatform(port_handle_t port, unsigned char* buffer, unsigned int readCount, int timeoutMilliseconds)
{
    serial_port_t* serialPort = (serial_port_t*)port;
    serialPortHandle* handle = (serialPortHandle*)serialPort->handle;
    if (!handle)
        return -1;

    for(unsigned int i = 0; i < readCount; ++i) {
        auto val = handle->uart->read();
        if(val < 0)
            return i;

        buffer[i] = (uint8_t)val;
    }

    return readCount;
}

static int serialPortReadPlatform(port_handle_t port, unsigned char* buffer, unsigned int readCount) {
    return serialPortReadTimeoutPlatform(port, buffer, readCount, 0);
}

static int serialPortAsyncReadPlatform(port_handle_t port, unsigned char* buffer, unsigned int readCount, pfnSerialPortAsyncReadCompletion completion)
{
    serial_port_t* serialPort = (serial_port_t*)port;
    serialPortHandle* handle = (serialPortHandle*)serialPort->handle;
    if (!handle)
        return -1;

    // no support for async, just call the completion right away
    int n = serialPortReadPlatform(port, buffer, readCount);
    completion(port, buffer, (n < 0 ? 0 : n), (n >= 0 ? 0 : n));

    return 1;
}

static int serialPortWritePlatform(port_handle_t port, const unsigned char* buffer, unsigned int writeCount)
{
    serial_port_t* serialPort = (serial_port_t*)port;
    serialPortHandle* handle = (serialPortHandle*)serialPort->handle;
    if (!handle)
        return -1;

    uint32_t bytes_written = handle->uart->write(buffer, writeCount);
    DEV_PRINTF("serialPortWritePlatform[%d], wrote[%" PRIu32 "]\r\n", writeCount, bytes_written);
    return bytes_written;
}

static int serialPortGetByteCountAvailableToReadPlatform(port_handle_t port)
{
    serial_port_t* serialPort = (serial_port_t*)port;
    serialPortHandle* handle = (serialPortHandle*)serialPort->handle;
    if (!handle)
        return -1;

    int bytesAvailable = handle->uart->available();
    return bytesAvailable;
}

static int serialPortGetByteCountAvailableToWritePlatform(port_handle_t port)
{

    serial_port_t* serialPort = (serial_port_t*)port;
    serialPortHandle* handle = (serialPortHandle*)serialPort->handle;
    if (!handle)
        return -1;

    int txspace = handle->uart->txspace();
    return txspace;
}

static int serialPortSleepPlatform(int sleepMilliseconds)
{
    auto scheduler = AP_HAL::get_HAL().scheduler;
    scheduler->delay(sleepMilliseconds);
    return 1;
}

int serialPortPlatformInit(port_handle_t port) // unsigned int portOptions
{
    serial_port_t* serialPort = (serial_port_t*)port;

    serialPortInit(port, 0, PORT_TYPE__UART | PORT_TYPE__COMM);

    // very important - the serial port must be initialized to zeros
    base_port_t tmp = { .pnum = portId(port), .ptype = portType(port) };

    // FIXME:  I really don't like this having to copy and clean, and copy back.  It shouldn't be necessary.
    char tmpName[64];
    memcpy(tmpName, serialPort->portName, _MIN(sizeof(serialPort->portName), sizeof(tmpName)));
    memset(serialPort, 0, sizeof(serial_port_t));
    memcpy(serialPort->portName, tmpName, _MIN(sizeof(serialPort->portName), sizeof(tmpName)));

    serialPort->base = tmp;

    serialPort->base.portName = serialPortName;
    serialPort->base.portClose = serialPortClose;
    serialPort->base.portFree = serialPortGetByteCountAvailableToWrite;
    serialPort->base.portAvailable = serialPortGetByteCountAvailableToRead;
    serialPort->base.portFlush = serialPortFlush;
    serialPort->base.portDrain = serialPortDrain;
    serialPort->base.portRead = serialPortRead;
    serialPort->base.portWrite = serialPortWrite;
    serialPort->base.portReadTimeout = (pfnPortReadTimeout)serialPortReadTimeout;

    serialPort->base.stats = (port_stats_t*)&serialPort->stats;

    if (portType(port) & PORT_TYPE__COMM)
        is_comm_port_init(COMM_PORT(port), NULL);

    // platform specific functions
    serialPort->pfnOpen = serialPortOpenPlatform;
    serialPort->pfnIsOpen = serialPortIsOpenPlatform;
    serialPort->pfnReadTimeout = serialPortReadTimeoutPlatform;
    serialPort->pfnAsyncRead = serialPortAsyncReadPlatform;
    serialPort->pfnFlush = serialPortFlushPlatform;
    serialPort->pfnDrain = serialPortDrainPlatform;
    serialPort->pfnClose = serialPortClosePlatform;
    serialPort->pfnGetByteCountAvailableToWrite = serialPortGetByteCountAvailableToWritePlatform;
    serialPort->pfnGetByteCountAvailableToRead = serialPortGetByteCountAvailableToReadPlatform;
    serialPort->pfnRead = serialPortReadPlatform;
    serialPort->pfnWrite = serialPortWritePlatform;
    serialPort->pfnSleep = serialPortSleepPlatform;

    return 0;
}

#endif // AP_EXTERNAL_AHRS_INERTIALSENSE_ENABLED
