/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_INERTIALSENSE_ENABLED

#include "serialPort.h"

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>

extern size_t strnlen(const char *s, size_t maxlen);

int SERIAL_PORT_DEFAULT_TIMEOUT = 2500;

void serialPortInit(port_handle_t port, int id, int type) {
    serial_port_t* serialPort = (serial_port_t*)port;
    serialPort->base.pnum = id;
    serialPort->base.ptype = type | PORT_FLAG__VALID;

    serialPort->base.stats = (port_stats_t*)&(serialPort->stats);

    serialPort->pfnOpen = serialPortOpen;
    serialPort->pfnClose = serialPortClose;
    serialPort->pfnReadTimeout = serialPortReadTimeout;
}

void serialPortSetOptions(port_handle_t port, uint32_t options)
{
    serial_port_t* serialPort = (serial_port_t*)port;
    if ((serialPort != NULL) && ((options & SERIAL_PORT_OPTIONS_MASK) == 0))
    {
        serialPort->options = options;
    }
}

void serialPortSetPort(port_handle_t port, const char* portName)
{
    serial_port_t* serialPort = (serial_port_t*)port;
    if ((serialPort != NULL) && (portName != NULL))
    {
        int portLength = (int)strnlen(portName, MAX_SERIAL_PORT_NAME_LENGTH);
        memcpy(serialPort->portName, portName, portLength);
        serialPort->portName[portLength] = '\0';
    }
}

const char *serialPortName(port_handle_t port) {
    serial_port_t* serialPort = (serial_port_t*)port;
    if (serialPort != NULL) {
        return serialPort->portName;
    }
    return NULL;
}

int serialPortOpen(port_handle_t port, void *uart)
{
    serial_port_t* serialPort = (serial_port_t*)port;
    if ((serialPort == 0) || (port == 0) || (serialPort->pfnOpen == 0))
    {
        if (serialPort && serialPort->pfnError) serialPort->pfnError(port, serialPort->errorCode, serialPort->error);
        return 0;
    }
    if (serialPort->pfnOpen(port, uart) != 1) {
        if (serialPort && serialPort->pfnError) serialPort->pfnError(port, serialPort->errorCode, serialPort->error);
        return 0;
    }
    serialPort->base.ptype |= PORT_FLAG__OPENED;
    return 1;
}

int serialPortOpenRetry(port_handle_t port, void *uart)
{
    serial_port_t* serialPort = (serial_port_t*)port;
    if ((serialPort == 0) || (serialPort->pfnOpen == 0))
    {
        if (serialPort && serialPort->pfnError) serialPort->pfnError(port, serialPort->errorCode, serialPort->error);
        return 0;
    }

    if (serialPortIsOpen(port))
        return 1;

    serialPortClose(port);  // Note that if port->handle is NULL, which usually indicates a closed port, we won't be able to close it. This is kind of superfluous.
    for (int retry = 0; retry < 5; retry++)
    {
        if (serialPortOpen(port, uart))
        {
            return 1;
        }
        if (serialPort->errorCode == ENOENT)
            break;  // don't retry if the port doesn't even exist
        serialPortSleep(port, 250);
    }
    if (serialPort && serialPort->pfnError) serialPort->pfnError(port, serialPort->errorCode, serialPort->error);
    if (serialPortIsOpen(port)) serialPortClose(port);
    return 0;
}

int serialPortIsOpen(port_handle_t port)
{
    serial_port_t* serialPort = (serial_port_t*)port;
    if ((serialPort == 0) || (serialPort->pfnIsOpen == 0))
    {
        if (serialPort && serialPort->pfnError) {
            serialPort->pfnError(port, -1, "port::IsOpen is not supported on this port.");
        }
        return 0;
    }
    return (serialPort->pfnIsOpen ? serialPort->pfnIsOpen(port) : 1);
}

int serialPortIsOpenQuick(port_handle_t port)
{
    serial_port_t* serialPort = (serial_port_t*)port;
    if ((serialPort == 0) || (serialPort->pfnIsOpen == 0))
    {
        if (serialPort && serialPort->pfnError) {
            serialPort->pfnError(port, -1, "port::IsOpen is not supported on this port.");
        }
        return 0;
    }

    if ((serialPort->handle != NULL) && (serialPort->errorCode == 0))
        return 1;

    return (serialPort->pfnIsOpen ? serialPort->pfnIsOpen(port) : 1);
}

int serialPortClose(port_handle_t port)
{
    serial_port_t* serialPort = (serial_port_t*)port;
    if ((serialPort == 0) || (serialPort->pfnClose == 0))
    {
        if (serialPort && serialPort->pfnError)
            serialPort->pfnError(port, serialPort->errorCode, serialPort->error);
        return 0;
    }
    serialPort->base.ptype &= ~PORT_FLAG__OPENED;       // safe to do before closing - because if the close fails, its fair the say the port is still invalid
    return serialPort->pfnClose(port);
}

int serialPortFlush(port_handle_t port)
{
    serial_port_t* serialPort = (serial_port_t*)port;
    if ((serialPort == 0) || (serialPort->pfnFlush == 0))
    {
        if (serialPort && serialPort->pfnError) serialPort->pfnError(port, serialPort->errorCode, serialPort->error);
        return 0;
    }
    return serialPort->pfnFlush(port);
}

int serialPortDrain(port_handle_t port, uint32_t timeoutMs)
{
    (void) timeoutMs;

    serial_port_t* serialPort = (serial_port_t*)port;
    if ((serialPort == 0) || (serialPort->pfnDrain == 0))
    {
        if (serialPort && serialPort->pfnError) serialPort->pfnError(port, serialPort->errorCode, serialPort->error);
        return 0;
    }
    return serialPort->pfnDrain(serialPort); // currently, our implementation ignores timeoutMs
}


int serialPortRead(port_handle_t port, unsigned char* buffer, unsigned int readCount)
{
    serial_port_t* serialPort = (serial_port_t*)port;
    if (!serialPort || !buffer || (readCount <= 0))
    {
        // don't report an error if readCount == 0; just return 0
        if (serialPort && serialPort->pfnError && (readCount > 0)) serialPort->pfnError(port, serialPort->errorCode, serialPort->error);
        return 0;
    }

    int count = serialPort->pfnRead(port, buffer, readCount);

    if (count < 0)
    {
        if (serialPort && serialPort->pfnError) serialPort->pfnError(port, serialPort->errorCode, serialPort->error);
        return 0;
    }

    return count;
}

int serialPortReadTimeout(port_handle_t port, unsigned char* buffer, unsigned int readCount, int timeoutMilliseconds)
{
    serial_port_t* serialPort = (serial_port_t*)port;
    if ((serialPort == 0) || (buffer == 0) || (readCount < 1))
    {
        if (serialPort && serialPort->pfnError) serialPort->pfnError(port, serialPort->errorCode, serialPort->error);
        return 0;
    }

    int count = serialPort->pfnReadTimeout(port, buffer, readCount, timeoutMilliseconds);
    if (count < 0)
    {
        if (serialPort && serialPort->pfnError) serialPort->pfnError(port, serialPort->errorCode, serialPort->error);
        return 0;
    }

    return count;
}

int serialPortReadTimeoutAsync(port_handle_t port, unsigned char* buffer, unsigned int readCount, pfnSerialPortAsyncReadCompletion completion)
{
    serial_port_t* serialPort = (serial_port_t*)port;
    if ((serialPort == 0) || (buffer == 0) || (readCount < 1) || (serialPort->pfnAsyncRead == 0) || (completion == 0))
    {
        if (serialPort && serialPort->pfnError) serialPort->pfnError(port, serialPort->errorCode, serialPort->error);
        return 0;
    }

    int count = serialPort->pfnAsyncRead(port, buffer, readCount, completion);
    if (count < 0)
    {
        return 0;
    }

    return count;
}

int serialPortReadLine(port_handle_t port, unsigned char* buffer, unsigned int bufferLength)
{
    // serial_port_t* serialPort = (serial_port_t*)port;
    return serialPortReadLineTimeout(port, buffer, bufferLength, SERIAL_PORT_DEFAULT_TIMEOUT);
}

int serialPortReadLineTimeout(port_handle_t port, unsigned char* buffer, unsigned int bufferLength, int timeoutMilliseconds)
{
    serial_port_t* serialPort = (serial_port_t*)port;
    if ((port == 0) || (buffer == 0) || (bufferLength < 8))
    {
        if (serialPort && serialPort->pfnError) serialPort->pfnError(port, serialPort->errorCode, serialPort->error);
        return 0;
    }

    int prevCR = 0;
    unsigned int bufferIndex = 0;
    unsigned char c;
    while (bufferIndex < bufferLength && serialPortReadCharTimeout(port, &c, timeoutMilliseconds) == 1)
    {
        buffer[bufferIndex++] = c;
        if (c == '\n' && prevCR)
        {
            // remove \r\n and null terminate and return count of chars
            buffer[bufferIndex -= 2] = '\0';
            return (int)bufferIndex;
        }
        prevCR = (c == '\r');
    }
    return -1;
}

int serialPortReadAscii(port_handle_t port, unsigned char* buffer, unsigned int bufferLength, unsigned char** asciiData)
{
    return serialPortReadAsciiTimeout(port, buffer, bufferLength, SERIAL_PORT_DEFAULT_TIMEOUT, asciiData);
}

int serialPortReadAsciiTimeout(port_handle_t port, unsigned char* buffer, unsigned int bufferLength, int timeoutMilliseconds, unsigned char** asciiData)
{
    int count = serialPortReadLineTimeout(port, buffer, bufferLength, timeoutMilliseconds);
    unsigned char* ptr = buffer;
    unsigned char* ptrEnd = buffer + count;
    while (*ptr != '$' && ptr < ptrEnd)
    {
        ptr++;
    }

    // if at least 8 chars available
    if (ptrEnd - ptr > 7)
    {
        if (asciiData != 0)
        {
            *asciiData = ptr;
        }
        int checksum = 0;
        int existingChecksum;

        // calculate checksum, skipping leading $ and trailing *XX\r\n
        unsigned char* ptrEndNoChecksum = ptrEnd - 3;
        while (++ptr < ptrEndNoChecksum)
        {
            checksum ^= *ptr;
        }

        if (*ptr == '*')
        {
            // read checksum from buffer, skipping the * char
            existingChecksum = strtol((void*)++ptr, NULL, 16);
            if (existingChecksum == checksum)
            {
                return count;
            }
        }
    }

    return -1;
}

int serialPortReadChar(port_handle_t port, unsigned char* c)
{
    return serialPortReadCharTimeout(port, c, SERIAL_PORT_DEFAULT_TIMEOUT);
}

int serialPortReadCharTimeout(port_handle_t port, unsigned char* c, int timeoutMilliseconds)
{
    return serialPortReadTimeout(port, c, 1, timeoutMilliseconds);
}

int serialPortWrite(port_handle_t port, const unsigned char* buffer, unsigned int writeCount)
{
    serial_port_t* serialPort = (serial_port_t*)port;
    if (!serialPort || !serialPort->handle || !buffer || (writeCount < 1))
    {
        if (serialPort && serialPort->pfnError) {
            if (!serialPort->handle) serialPort->pfnError(port, ENOENT, strerror(ENOENT));
            else serialPort->pfnError(port, EINVAL, strerror(EINVAL));
        }
        return 0;
    }

    int count = serialPort->pfnWrite(port, buffer, writeCount);
    if (count < 0)
    {
        if (serialPort->pfnError) serialPort->pfnError(port, serialPort->errorCode, serialPort->error);
        return 0;
    }

    return count;
}

int serialPortWriteLine(port_handle_t port, const unsigned char* buffer, unsigned int writeCount)
{
    int count = serialPortWrite(port, buffer, writeCount);
    if (count == (int)writeCount)
        count += serialPortWrite(port, (unsigned char*)"\r\n", 2);
    return count;
}

int serialPortWriteAscii(port_handle_t port, const char* buffer, unsigned int bufferLength)
{
    serial_port_t* serialPort = (serial_port_t*)port;
    if (!serialPort || !serialPort->handle || !buffer || (bufferLength < 2))
    {
        if (!serialPort || !serialPort->handle) serialPort->pfnError(port, ENOENT, strerror(ENOENT));
        else serialPort->pfnError(port, EINVAL, strerror(EINVAL));
        return 0;
    }

    int checkSum = 0;
    const unsigned char* ptr = (const unsigned char*)buffer;
    int count = 0;

    if (*buffer == '$')
    {
        ptr++;
        bufferLength--;
    }
    else
    {
        count += serialPortWrite(port, (const unsigned char*)"$", 1);
    }

    const unsigned char* ptrEnd = ptr + bufferLength;
    unsigned char buf[16];

    count += serialPortWrite(port, (const unsigned char*)buffer, bufferLength);

    while (ptr != ptrEnd)
    {
        checkSum ^= *ptr++;
    }

#ifdef _MSC_VER

#pragma warning(push)
#pragma warning(disable: 4996)

#endif

    snprintf((char*)buf, sizeof(buf), "*%.2x\r\n", checkSum);

#ifdef _MSC_VER

#pragma warning(pop)

#endif

    count += serialPortWrite(port, buf, 5);

    return count;
}

int serialPortWriteAndWaitFor(port_handle_t port, const unsigned char* buffer, unsigned int writeCount, const unsigned char* waitFor, unsigned int waitForLength)
{
    return serialPortWriteAndWaitForTimeout(port, buffer, writeCount, waitFor, waitForLength, SERIAL_PORT_DEFAULT_TIMEOUT);
}

int serialPortWriteAndWaitForTimeout(port_handle_t port, const unsigned char* buffer, unsigned int writeCount, const unsigned char* waitFor, unsigned int waitForLength, const int timeoutMilliseconds)
{
    if (serialPortWrite(port, buffer, writeCount) != (int)writeCount)
    {
        return 0;
    }

    return serialPortWaitForTimeout(port, waitFor, waitForLength, timeoutMilliseconds);
}

int serialPortWaitFor(port_handle_t port, const unsigned char* waitFor, unsigned int waitForLength)
{
    return serialPortWaitForTimeout(port, waitFor, waitForLength, SERIAL_PORT_DEFAULT_TIMEOUT);
}

int serialPortWaitForTimeout(port_handle_t port, const unsigned char* waitFor, unsigned int waitForLength, int timeoutMilliseconds)
{
    serial_port_t* serialPort = (serial_port_t*)port;
    if (!serialPort) return 0;
    if (!serialPort->handle) {
        if (serialPort->pfnError) serialPort->pfnError(port, EBADF, strerror(EBADF));
        return 0;
    }

    if ((waitFor == 0) || (waitForLength < 1))
    {
        return 1;
    }
    else if (waitForLength > 128)
    {
        if (serialPort->pfnError) serialPort->pfnError(port, EBADF, strerror(EBADF));
        return 0;
    }

    unsigned char buf[128] = { 0 };
    int count = serialPortReadTimeout(port, buf, waitForLength, timeoutMilliseconds);
    if ((count == (int)waitForLength) && !memcmp(buf, waitFor, waitForLength))
    {
        return 1;
    }
    return 0;
}

int serialPortGetByteCountAvailableToRead(port_handle_t port)
{
    serial_port_t* serialPort = (serial_port_t*)port;
    if (serialPort == 0 || serialPort->handle == 0 || serialPort->pfnGetByteCountAvailableToRead == 0)
    {
        return 0;
    }

    return serialPort->pfnGetByteCountAvailableToRead(port);
}

int serialPortGetByteCountAvailableToWrite(port_handle_t port)
{
    serial_port_t* serialPort = (serial_port_t*)port;
    if (serialPort == 0 || serialPort->handle == 0 || serialPort->pfnGetByteCountAvailableToWrite == 0)
    {
        return 0;
    }

    return serialPort->pfnGetByteCountAvailableToWrite(port);
}

int serialPortSleep(port_handle_t port, int sleepMilliseconds)
{
    serial_port_t* serialPort = (serial_port_t*)port;
    if (serialPort == 0 || serialPort->pfnSleep == 0)
    {
        return 0;
    }

    return serialPort->pfnSleep(sleepMilliseconds);
}

int serialPortSetErrorCB(port_handle_t port, pfnSerialPortOnErrorCB onErrorCb)
{
    serial_port_t* serialPort = (serial_port_t*)port;
    if (serialPort == 0)
    {
        return -1;
    }

    serialPort->pfnError = onErrorCb;
    return 0;
}

#endif // AP_EXTERNAL_AHRS_INERTIALSENSE_ENABLED
