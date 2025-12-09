/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_INERTIALSENSE_ENABLED

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdint.h>

#include "ISComm.h"

extern int SERIAL_PORT_DEFAULT_TIMEOUT;

#define MAX_SERIAL_PORT_NAME_LENGTH 63

// Standard Baud Rates - FTDI Functional.   // Bit period = 1/baudrate, Actual baud (FTDI,AVR,ARM)
#define BAUDRATE_300        300             // 3333 us
#define BAUDRATE_600        600             // 1667 us
#define BAUDRATE_1200       1200            //  833 us
#define BAUDRATE_2400       2400            //  417 us
#define BAUDRATE_4800       4800            //  208 us
#define BAUDRATE_9600       9600            //  104 us
#define BAUDRATE_19200      19200           //   52 us
#define BAUDRATE_38400      38400           //   26 us
#define BAUDRATE_57600      57600           //   17 us
#define BAUDRATE_115200     115200          // 8680 ns
#define BAUDRATE_230400     230400          // 4340 ns
#define BAUDRATE_460800     460800          // 2170 ns
#define BAUDRATE_921600     921600          // 1085 ns
#define BAUDRATE_1000000    1000000         // 1000 ns
#define BAUDRATE_1220000    1220000         //  820 ns
#define BAUDRATE_1440000    1440000         //  794 ns
#define BAUDRATE_1500000    1500000         //  667 ns    (FTDI 1520, AFR 1500)
#define BAUDRATE_2000000    2000000         //  500 ns    (FTDI 2080, AVR/ARM 2016)
#define BAUDRATE_3000000    3000000         //  333 ns    (FTDI 3150, AVR/ARM 3030)

enum eSerialPortOptions
{
    OPT_PARITY_NONE = 0x0,
    OPT_PARITY_ODD = 0x1,
    OPT_PARITY_EVEN = 0x2,
    OPT_PARITY_MASK = 0x3,

    SERIAL_PORT_OPTIONS_MASK = OPT_PARITY_MASK,
};

typedef int(*pfnSerialPortOpen)(port_handle_t port, void *uart);
typedef int(*pfnSerialPortIsOpen)(port_handle_t port);
typedef int(*pfnSerialPortRead)(port_handle_t port, unsigned char* buf, unsigned int len);
typedef int(*pfnSerialPortReadTimeout)(port_handle_t port, unsigned char* buf, unsigned int len, int timeoutMs);
typedef void(*pfnSerialPortAsyncReadCompletion)(port_handle_t port, unsigned char* buf, unsigned int len, int errorCode);
typedef int(*pfnSerialPortAsyncRead)(port_handle_t port, unsigned char* buf, unsigned int len, pfnSerialPortAsyncReadCompletion completion);
typedef int(*pfnSerialPortWrite)(port_handle_t port, const unsigned char* buf, unsigned int len);
typedef int(*pfnSerialPortClose)(port_handle_t port);
typedef int(*pfnSerialPortFlush)(port_handle_t port);
typedef int(*pfnSerialPortGetByteCountAvailableToRead)(port_handle_t port);
typedef int(*pfnSerialPortGetByteCountAvailableToWrite)(port_handle_t port);
typedef int(*pfnSerialPortSleep)(int sleepMilliseconds);
typedef int(*pfnSerialPortOnErrorCB)(port_handle_t port, int errCode, const char *errMsg);

// Allows communicating over a serial port
struct serial_port_s
{
    // base "implementation"
    union {
        base_port_t base;
        comm_port_t comm;
    };

    port_monitor_set_t stats;

    rmci_t rmci;
    uint8_t rmciUPMcnt[DID_COUNT];
    uint8_t rmciNMEAcnt[NMEA_MSG_ID_COUNT];

    // platform specific handle
    void* handle;

    // the port name (do not modify directly)
    char portName[MAX_SERIAL_PORT_NAME_LENGTH + 1];

    // the current (or expected) baud rate to communicate at
    int baudRate;

    // non-zero if this port is configured for blocking calls
    int blocking;

    // latest errno that was reported from an operation on this port
    int errorCode;

    // optional error buffer to store errors
    char* error;

    // length of error
    int errorLength;

    // Number of bytes sent
    // int txBytes;    // are these still needed?

    // Number of bytes received
    // int rxBytes;    // are these still needed?

    // Options for encoding like parity, stop bits, etc. (see eSerialPortOptions)
    uint32_t options;

    // open the serial port
    pfnSerialPortOpen pfnOpen;

    // is the serial port open?
    pfnSerialPortIsOpen pfnIsOpen;

    // read data synchronously
    pfnSerialPortRead pfnRead;

    // read data synchronously w/ timeout
    pfnSerialPortReadTimeout pfnReadTimeout;

    // read data asynchronously
    pfnSerialPortAsyncRead pfnAsyncRead;

    // write data synchronously
    pfnSerialPortWrite pfnWrite;

    // close the serial port
    pfnSerialPortClose pfnClose;

    // discard all data from all buffers
    pfnSerialPortFlush pfnFlush;

    // block until all queued TX data has been sent
    pfnSerialPortFlush pfnDrain;

    // get number of bytes in the receive buffer that can be read
    pfnSerialPortGetByteCountAvailableToRead pfnGetByteCountAvailableToRead;

    // get the number of available bytes in the send buffer
    pfnSerialPortGetByteCountAvailableToWrite pfnGetByteCountAvailableToWrite;

    // sleep for a specified number of milliseconds
    pfnSerialPortSleep pfnSleep;

    pfnSerialPortOnErrorCB pfnError;
};

typedef struct serial_port_s serial_port_t;
#define SERIAL_PORT(n)  ((serial_port_t*)n)


void serialPortInit(port_handle_t, int id, int type);

// set the port name for a serial port, in case you are opening it later
void serialPortSetPort(port_handle_t port, const char* portName);

/**
 * returns the name associated with this port (this is usually the OS's identifier)
 * @param port
 * @return returns the name associated with this port (this is usually the OS's identifier)
 */
const char *serialPortName(port_handle_t port);

/**
 * open a serial port
 * portName is null terminated, i.e. COM1\0, COM2\0, etc.
 * use blocking = 0 when data is being streamed from the serial port rapidly and blocking = 1 for
 * uses such as a boot loader where a write would then require n bytes to be read in a single operation.
 * blocking simply determines the default timeout value of the serialPortRead function
 * @param port
 * @param portName
 * @param baudRate
 * @param blocking
 * @return 1 if success, 0 if failure
 */
int serialPortOpen(port_handle_t port, void *uart);


/**
 * open a serial port with retry
 * portName is null terminated, i.e. COM1\0, COM2\0, etc.
 * use blocking = 0 when data is being streamed from the serial port rapidly and blocking = 1 for
 * uses such as a boot loader where a write would then require n bytes to be read in a single operation.
 * blocking simply determines the default timeout value of the serialPortRead function
 * @param port
 * @param portName
 * @param baudRate
 * @param blocking
 * @return 1 if success, 0 if failure
 */
int serialPortOpenRetry(port_handle_t port, void *uart);

/**
 * check if the port is open
 * @param port
 * @return 1 if open, 0 if not open
 */
int serialPortIsOpen(port_handle_t port);

/**
 * check if the port is open, but avoids an expensive OS/kernel call is possible.
 * If the internal handle is NOT null, and there are recent errors on the port
 * this function will return true, indicating that the port is open.  However,
 * it should be noted that the port may still be closed by the OS or another
 * mechanism which may not be reflected in the local state, which could cause
 * this to report incorrectly and then leading to a future error state when
 * operating on the closed port.
 * @param port
 * @return 1 if open, 0 if not open
 */
int serialPortIsOpenQuick(port_handle_t port);

/**
 * close the serial port - this object can be re-used by calling open again
 * @param port
 * @return 1 if closed, 0 if the port was not closed
 */
int serialPortClose(port_handle_t port);

/**
 * clear all buffers and pending reads and writes
 * @param port
 * @return 1 if success, 0 if failure
 */
int serialPortFlush(port_handle_t port);

/**
 * blocks until all pending TX writes have completed, and the TX buffer is empty.
 * @param port
 * @param timeoutMs the number of milliseconds to wait, at most before data is discarded
 * @return 1 if success, 0 if failure
 */
int serialPortDrain(port_handle_t port, uint32_t timeoutMs);

/**
 * read up to readCount bytes into buffer
 * call is forwarded to serialPortReadTimeout with timeoutMilliseconds of 0 for non-blocking, or SERIAL_PORT_DEFAULT_TIMEOUT for blocking.
 * @param port
 * @param buffer
 * @param readCount
 * @return number of bytes read which is less than or equal to readCount.
 */
int serialPortRead(port_handle_t port, unsigned char* buffer, unsigned int readCount);

/**
 * read up to thue number of bytes requested
 * @param port
 * @param buffer
 * @param readCount
 * @param timeoutMilliseconds
 * @return number of bytes read which is less than or equal to readCount
 */
int serialPortReadTimeout(port_handle_t port, unsigned char* buffer, unsigned int readCount, int timeoutMilliseconds);

/**
 * start an async read - not all platforms will support an async read and may call the callback function immediately
 * reads up to readCount bytes into buffer
 * buffer must exist until callback is executed, if it needs to be freed, free it in the callback or later
 * @param port
 * @param buffer
 * @param readCount
 * @param callback
 * @return 1 if success, 0 if failed to start async operation
 */
int serialPortReadTimeoutAsync(port_handle_t port, unsigned char* buffer, unsigned int readCount, pfnSerialPortAsyncReadCompletion callback);

/**
 * read up until a \r\n sequence has been read
 * buffer will not contain \r\n sequence
 * @param port
 * @param buffer
 * @param bufferLength
 * @return number of bytes read or -1 if timeout or buffer overflow, count does not include the null terminator
 */
int serialPortReadLine(port_handle_t port, unsigned char* buffer, unsigned int bufferLength);

/**
 * read up until a \r\n sequence has been read
 * result will not contain \r\n sequence
 * @param port
 * @param buffer
 * @param bufferLength
 * @param timeoutMilliseconds
 * @return number of bytes read or -1 if timeout or buffer overflow, count does not include the null terminator
 */
int serialPortReadLineTimeout(port_handle_t port, unsigned char* buffer, unsigned int bufferLength, int timeoutMilliseconds);

/**
 * read ASCII data (starts with $ and ends with \r\n, based on NMEA format)
 * will ignore data that fails checksum
 * asciiData gets set to start of ASCII data
 * @param port
 * @param buffer
 * @param bufferLength
 * @param asciiData
 * @return -1 if timeout or buffer overflow or checksum failure
 */
int serialPortReadAscii(port_handle_t port, unsigned char* buffer, unsigned int bufferLength, unsigned char** asciiData);

/**
 * read ASCII data (starts with $ and ends with \r\n, based on NMEA format)
 * will ignore data that fails checksum
 * asciiData gets set to start of ASCII data
 * @param port
 * @param buffer
 * @param bufferLength
 * @param timeoutMilliseconds
 * @param asciiData
 * @return -1 if timeout or buffer overflow or checksum failure
 */
int serialPortReadAsciiTimeout(port_handle_t port, unsigned char* buffer, unsigned int bufferLength, int timeoutMilliseconds, unsigned char** asciiData);

// read one char, waiting SERIAL_PORT_DEFAULT_TIMEOUT milliseconds to get a char
int serialPortReadChar(port_handle_t port, unsigned char* c);

// read one char, waiting timeoutMilliseconds to get a char, returns number of chars read
int serialPortReadCharTimeout(port_handle_t port, unsigned char* c, int timeoutMilliseconds);

// write, returns the number of bytes written
int serialPortWrite(port_handle_t port, const unsigned char* buffer, unsigned int writeCount);

// write with a \r\n added at the end, \r\n should not be part of buffer, returns the number of bytes written
int serialPortWriteLine(port_handle_t port, const unsigned char* buffer, unsigned int writeCount);

// write ascii data - if buffer does not start with $, a $ will be written first, followed by buffer, followed by *xx\r\n, where xx is a two hex character checksum
int serialPortWriteAscii(port_handle_t port, const char* buffer, unsigned int bufferLength);

// write and wait for a response, returns 1 if success, 0 if failure
int serialPortWriteAndWaitFor(port_handle_t port, const unsigned char* buffer, unsigned int writeCount, const unsigned char* waitFor, unsigned int waitForLength);
int serialPortWriteAndWaitForTimeout(port_handle_t port, const unsigned char* buffer, unsigned int writeCount, const unsigned char* waitFor, unsigned int waitForLength, const int timeoutMilliseconds);

// wait for a response, returns 0 if failure, 1 if success
int serialPortWaitFor(port_handle_t port, const unsigned char* waitFor, unsigned int waitForLength);
int serialPortWaitForTimeout(port_handle_t port, const unsigned char* waitFor, unsigned int waitForLength, int timeoutMilliseconds);

// get available bytes in the receive buffer
int serialPortGetByteCountAvailableToRead(port_handle_t port);

// get available bytes in the send buffer
int serialPortGetByteCountAvailableToWrite(port_handle_t port);

// sleep for the specified number of milliseconds if supported, returns 1 if success, 0 if failed to sleep
int serialPortSleep(port_handle_t port, int sleepMilliseconds);

// Set the port options
void serialPortSetOptions(port_handle_t port, uint32_t options);

// Set callback for error events
int serialPortSetErrorCB(port_handle_t port, pfnSerialPortOnErrorCB onErrorCb);


#ifdef __cplusplus
}
#endif

#endif // AP_EXTERNAL_AHRS_INERTIALSENSE_ENABLED
