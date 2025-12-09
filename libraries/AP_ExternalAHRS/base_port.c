/**
 * @file base_port.c
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 5/9/25.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_INERTIALSENSE_ENABLED

#include "base_port.h"

#include <errno.h>
#include <time.h>

extern uint32_t millis();
// extern int nanosleep(const struct timespec *duration, struct timespec *_Nullable rem);

void SLEEP_US(int us);
void SLEEP_MS(int ms);

/** System time in milliseconds */
unsigned int current_timeMs() {
    return millis();
}


void SLEEP_US(int us) {
    // struct timespec ts;
    // ts.tv_sec = 0;
    // ts.tv_nsec = us*1000UL;
    // while (nanosleep(&ts, &ts) == -1 && errno == EINTR);
}

void SLEEP_MS(int ms) {
    // struct timespec ts;
    // ts.tv_sec = 0;
    // ts.tv_nsec = ms*1000UL*1000UL;
    // while (nanosleep(&ts, &ts) == -1 && errno == EINTR);
}

/**
 * General purpose blocking "read n bytes" with a timeout in the event that n bytes aren't received within the alotted timeout
 * @param port
 * @param buffer
 * @param readCount
 * @param timeoutMs
 * @return number of bytes read. If the number of bytes read is less than the readCount value, it should be assumed that a timeout occurred
 */
int portReadTimeout_internal(port_handle_t port, uint8_t* buffer, unsigned int readCount, unsigned int timeoutMs)
{
    if (!portIsValid(port) || (buffer == 0) || (readCount <= 0))
        return 0;

    uint32_t timeout = current_timeMs() + timeoutMs;
    uint16_t bytesPending = portAvailable(port);
    while ((bytesPending < readCount) && (current_timeMs() > timeout)) {
        SLEEP_MS(timeoutMs / 4);
        bytesPending = portAvailable(port);
    }

    return portRead(port, buffer, bytesPending < readCount ? bytesPending : readCount);
}

/**
 * Waits timeoutMilliseconds for a sequence of waitForLength bytes matching waitFor to be read from a part.
 * No data is retained or buffered while waiting. If the sequence is read within the timeout period, this returns
 * true, otherwise false. NOTE: This is a blocking call, and any data read while looking for a match is discarded.
 * @param port the port monitor for the anticipated bytes
 * @param waitFor the sequence of bytes to wait for, not to exceed 128 bytes
 * @param waitForLength the number of bytes in waitFor
 * @param timeoutMs the maximum number of milliseconds to wait
 * @return true (non-zero) if the waitFor sequence is received in time, otherwise false (zero)
 */
int portWaitForTimeout(port_handle_t port, const unsigned char* waitFor, unsigned int waitForLength, unsigned int timeoutMs)
{
    if ((port == 0) || (waitFor == 0) || (waitForLength < 1) || (waitForLength >= 128))
        return 0;

    unsigned char buf[132] = { 0 }; // note that we are padding this slightly (4 bytes)
    uint8_t* bPtr = buf;
    int bytesWaiting = 0;

    uint32_t timeout = current_timeMs() + timeoutMs;
    while (current_timeMs() < timeout) {
        bytesWaiting = portAvailable(port);
        if (bytesWaiting > 0) {
            // append the read bytes into our working buffer, but don't be greedy about it (only take what we need)
            uint8_t buffSpace = sizeof(buf) - (bPtr - buf);
            uint8_t maxRead = buffSpace < waitForLength ? buffSpace : waitForLength;
            int bytesRead = portRead(port, bPtr, maxRead);   // note that we don't read bytesWaiting, but just enough to fill our buffer
            bPtr += bytesRead;

            // now scan (hopefully quickly) to see if there is any bytes of interest in the data we just read
            unsigned int nMatch = 0;         // number of bytes from 'waitFor' that have matches sequentially
            uint8_t *mPtr = NULL;   // a pointer into buf where the first matching character is found; we'll purge upto this point if we don't find a match
            for (uint8_t* sPtr = buf; sPtr < bPtr; sPtr++) {
                if (*sPtr == waitFor[nMatch]) {
                    if (nMatch == 0)
                        mPtr = sPtr;
                    if (nMatch >= waitForLength-1)
                        return 1;       // success, we match all 'waitForLength' bytes
                    nMatch++;
                } else {
                    // failed to match the full sequence, so reset back to looking for the first char
                    nMatch = 0;
                    mPtr = NULL;
                }
            }
            if (mPtr && nMatch) {
                // we found at least one matching character (which wasn't reset by a subsequent non-matching character),
                // but still no complete match, so let's dump that from our working buffer, and try again.
                memmove(buf, mPtr, sizeof(buf) - (mPtr - buf));
            }
        } else if (bytesWaiting < 0) {
            return 0;   // error while reading
        } else {
            SLEEP_US(1);
        }
    }

    return 0;
}

int portWaitFor(port_handle_t port, const unsigned char* waitFor, unsigned int waitForLength)
{
    // serial_port_t* serialPort = (serial_port_t*)port;
    return portWaitForTimeout(port, waitFor, waitForLength, PORT_DEFAULT_TIMEOUT);
}

int portReadCharTimeout(port_handle_t port, unsigned char* c, unsigned int timeoutMs)
{
    return portReadTimeout(port, c, 1, timeoutMs);
}

int portReadChar(port_handle_t port, unsigned char* c)
{
    return portReadCharTimeout(port, c, PORT_DEFAULT_TIMEOUT);
}

/**
 * Reads up to either a newline sequence (CRLF) or bufferLength characters unless a timeout
 *  occurs waiting for either to occur
 * @param port the port to read from
 * @param buffer the buffer to read bytes into
 * @param bufferLength the maximum number of bytes that can be placed into the buffer
 * @param timeoutMs the maximum time in milliseconds to wait for the data to arrive
 * @return the number of bytes read, including the newline or -1 if a timeout occurs
 */
int portReadLineTimeout(port_handle_t port, unsigned char* buffer, unsigned int bufferLength, unsigned int timeoutMs)
{
    if (!portIsValid(port))
        return PORT_ERROR__INVALID;

    int prevCR = 0;
    unsigned int bufferIndex = 0;
    unsigned char c;
    uint32_t startMs = current_timeMs();
    do {
        if (portAvailable(port) == 0) {
            SLEEP_US(10);
            continue;
        }

        if (portRead(port, &c, 1) == 1)
        {
            buffer[bufferIndex++] = c;
            if ((c == '\n' && prevCR) || (bufferIndex >= bufferLength))
            {
                // remove \r\n and null terminate and return count of chars
                buffer[bufferIndex -= 2] = '\0';
                return (int)bufferIndex;
            }
            prevCR = (c == '\r');
        }
    } while ((current_timeMs() - startMs) < timeoutMs);
    return -1;
}

int portReadLine(port_handle_t port, unsigned char* buffer, unsigned int bufferLength)
{
    // serial_port_t* serialPort = (serial_port_t*)port;
    return portReadLineTimeout(port, buffer, bufferLength, PORT_DEFAULT_TIMEOUT);
}

int portReadAsciiTimeout(port_handle_t port, unsigned char* buffer, unsigned int bufferLength, unsigned int timeoutMs, unsigned char** asciiData)
{
    int count = portReadLineTimeout(port, buffer, bufferLength, timeoutMs);
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

int portReadAscii(port_handle_t port, unsigned char* buffer, unsigned int bufferLength, unsigned char** asciiData)
{
    return portReadAsciiTimeout(port, buffer, bufferLength, PORT_DEFAULT_TIMEOUT, asciiData);
}

int portWriteLine(port_handle_t port, const unsigned char* buffer, unsigned int writeCount)
{
    if (!portIsValid(port)) return 0;

    int count = portWrite(port, buffer, writeCount);
    count += portWrite(port, (unsigned char*)"\r\n", 2);
    return count;
}

int portWriteAscii(port_handle_t port, const char* buffer, unsigned int bufferLength)
{
    if (!portIsValid(port)) return 0;

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
        count += portWrite(port, (const unsigned char*)"$", 1);
    }

    const unsigned char* ptrEnd = ptr + bufferLength;
    unsigned char buf[16];

    count += portWrite(port, (const unsigned char*)buffer, bufferLength);

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

    count += portWrite(port, buf, 5);

    return count;
}

int portWriteAndWaitForTimeout(port_handle_t port, const unsigned char* buffer, unsigned int writeCount, const unsigned char* waitFor, unsigned int waitForLength, const unsigned int timeoutMs)
{
    if (!portIsValid(port))
        return 0;

    int actuallyWrittenCount = portWrite(port, buffer, writeCount);
    if (actuallyWrittenCount != (int)writeCount)
        return 0;

    return portWaitForTimeout(port, waitFor, waitForLength, timeoutMs);
}

int portWriteAndWaitFor(port_handle_t port, const unsigned char* buffer, unsigned int writeCount, const unsigned char* waitFor, unsigned int waitForLength)
{
    //serial_port_t* serialPort = (serial_port_t*)port;
    return portWriteAndWaitForTimeout(port, buffer, writeCount, waitFor, waitForLength, PORT_DEFAULT_TIMEOUT);
}

#endif // AP_EXTERNAL_AHRS_INERTIALSENSE_ENABLED
