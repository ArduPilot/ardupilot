/*
  port for a script to access from a device perspective
 */

#include "AP_Scripting_config.h"

#if AP_SCRIPTING_ENABLED && AP_SCRIPTING_SERIALDEVICE_ENABLED

#include "AP_Scripting_SerialDevice.h"

#include <AP_Math/AP_Math.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

#ifndef AP_SCRIPTING_SERIALDEVICE_MIN_TXSIZE
#define AP_SCRIPTING_SERIALDEVICE_MIN_TXSIZE 2048
#endif

#ifndef AP_SCRIPTING_SERIALDEVICE_MIN_RXSIZE
#define AP_SCRIPTING_SERIALDEVICE_MIN_RXSIZE 2048
#endif

AP_Scripting_SerialDevice *AP_Scripting_SerialDevice::_singleton;

const AP_Param::GroupInfo AP_Scripting_SerialDevice::var_info[] = {
    // These indices and suffixes match AP_Scripting::var_info. AP_Vehicle uses
    // this minimal group only when Rust is the sole scripting runtime.
    AP_GROUPINFO_FLAGS("SDEV_EN", 15, AP_Scripting_SerialDevice, enable, 0, AP_PARAM_FLAG_ENABLE),
    AP_GROUPINFO("SDEV1_PROTO", 16, AP_Scripting_SerialDevice, ports[0].state.protocol, -1),
#if AP_SCRIPTING_SERIALDEVICE_NUM_PORTS > 1
    AP_GROUPINFO("SDEV2_PROTO", 17, AP_Scripting_SerialDevice, ports[1].state.protocol, -1),
#endif
#if AP_SCRIPTING_SERIALDEVICE_NUM_PORTS > 2
    AP_GROUPINFO("SDEV3_PROTO", 18, AP_Scripting_SerialDevice, ports[2].state.protocol, -1),
#endif
    AP_GROUPEND
};

AP_Scripting_SerialDevice::AP_Scripting_SerialDevice()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("Scripting serial device must be a singleton");
    }
#endif
    _singleton = this;
}

/*
  initialise scripting serial ports
*/
void AP_Scripting_SerialDevice::init(void)
{
    if (enable == 0) {
        return;
    }

    // Zero is reserved for malformed handles. The seven-bit value is encoded
    // into Rust's revalidated simulated-port selector.
    _generation = (_generation % 0x7fU) + 1U;

    for (uint8_t i=0; i<ARRAY_SIZE(ports); i++) {
        auto &p = ports[i];
        p.state.idx = AP_SERIALMANAGER_SCR_PORT_1 + i;
        p.init();
        AP::serialmanager().register_port(&p);
    }
}

AP_Scripting_SerialDevice::Port *AP_Scripting_SerialDevice::find_port(int8_t protocol,
                                                                      uint16_t instance,
                                                                      uint8_t generation)
{
    if (enable == 0 || generation == 0 || generation != _generation) {
        return nullptr;
    }
    for (auto &port : ports) {
        if (port.state.protocol == protocol && instance-- == 0) {
            WITH_SEMAPHORE(port.sem);
            port._native_accessed = true;
            return &port;
        }
    }
    return nullptr;
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
    // Native selectors can outlive a Lua restart and have no close operation.
    if (_native_accessed) {
        return;
    }
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
