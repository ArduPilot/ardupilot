/*
  port for a script to access from a device perspective
 */

#include "AP_Scripting_config.h"

#if AP_SCRIPTING_ENABLED && AP_SCRIPTING_SERIALDEVICE_ENABLED

#include "AP_Scripting.h"

#include <AP_Math/AP_Math.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

#ifndef AP_SCRIPTING_SERIALDEVICE_MIN_TXSIZE
#define AP_SCRIPTING_SERIALDEVICE_MIN_TXSIZE 2048
#endif

#ifndef AP_SCRIPTING_SERIALDEVICE_MIN_RXSIZE
#define AP_SCRIPTING_SERIALDEVICE_MIN_RXSIZE 2048
#endif

/*
  initialise scripting serial ports
*/
void AP_Scripting_SerialDevice::init(void)
{
    if (enable == 0) {
        return;
    }

    for (uint8_t i=0; i<ARRAY_SIZE(ports); i++) {
        auto &p = ports[i];
        p.state.idx = AP_SERIALMANAGER_SCR_PORT_1 + i;
        p.init();
        AP::serialmanager().register_port(&p);
    }
}

void AP_Scripting_SerialDevice::clear(void)
{
    for (auto &p : ports) {
        p.clear();
    }
}

/*
  initialise port
 */
void AP_Scripting_SerialDevice::Port::init(void)
{
    begin(1000000, 0, 0); // assume 1MBaud rate even though it's a bit meaningless
}

void AP_Scripting_SerialDevice::Port::clear(void)
{
    WITH_SEMAPHORE(sem);
    if (readbuffer) {
        readbuffer->clear();
    }
    if (writebuffer) {
        writebuffer->clear();
    }
}

size_t AP_Scripting_SerialDevice::Port::device_write(const uint8_t *buffer, size_t size)
{
    WITH_SEMAPHORE(sem);
    if (readbuffer) {
        return readbuffer->write(buffer, size);
    }
    return 0;
}

ssize_t AP_Scripting_SerialDevice::Port::device_read(uint8_t *buffer, uint16_t count)
{
    WITH_SEMAPHORE(sem);
    if (writebuffer) {
        return writebuffer->read(buffer, count);
    }
    return 0;
}

uint32_t AP_Scripting_SerialDevice::Port::device_available(void)
{
    WITH_SEMAPHORE(sem);
    if (writebuffer) {
        return writebuffer->available();
    }
    return 0;
}

/*
  available space in outgoing buffer
 */
uint32_t AP_Scripting_SerialDevice::Port::txspace(void)
{
    WITH_SEMAPHORE(sem);
    return writebuffer != nullptr ? writebuffer->space() : 0;
}

void AP_Scripting_SerialDevice::Port::_begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
    rxS = MAX(rxS, AP_SCRIPTING_SERIALDEVICE_MIN_RXSIZE);
    txS = MAX(txS, AP_SCRIPTING_SERIALDEVICE_MIN_TXSIZE);
    init_buffers(rxS, txS);
}

size_t AP_Scripting_SerialDevice::Port::_write(const uint8_t *buffer, size_t size)
{
    WITH_SEMAPHORE(sem);
    return writebuffer != nullptr ? writebuffer->write(buffer, size) : 0;
}

ssize_t AP_Scripting_SerialDevice::Port::_read(uint8_t *buffer, uint16_t count)
{
    WITH_SEMAPHORE(sem);
    return readbuffer != nullptr ? readbuffer->read(buffer, count) : -1;
}

uint32_t AP_Scripting_SerialDevice::Port::_available()
{
    WITH_SEMAPHORE(sem);
    return readbuffer != nullptr ? readbuffer->available() : 0;
}


bool AP_Scripting_SerialDevice::Port::_discard_input()
{
    WITH_SEMAPHORE(sem);
    if (readbuffer != nullptr) {
        readbuffer->clear();
    }
    return true;
}

/*
  initialise read/write buffers
 */
bool AP_Scripting_SerialDevice::Port::init_buffers(const uint32_t size_rx, const uint32_t size_tx)
{
    if (size_tx == last_size_tx &&
        size_rx == last_size_rx) {
        return true;
    }
    WITH_SEMAPHORE(sem);
    if (readbuffer == nullptr) {
        readbuffer = NEW_NOTHROW ByteBuffer(size_rx);
    } else {
        readbuffer->set_size_best(size_rx);
    }
    if (writebuffer == nullptr) {
        writebuffer = NEW_NOTHROW ByteBuffer(size_tx);
    } else {
        writebuffer->set_size_best(size_tx);
    }
    last_size_rx = size_rx;
    last_size_tx = size_tx;
    return readbuffer != nullptr && writebuffer != nullptr;
}

#endif // AP_SCRIPTING_ENABLED && AP_SCRIPTING_SERIALDEVICE_ENABLED
