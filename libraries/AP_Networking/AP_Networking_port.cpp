/*
  class for networking mapped ports
 */

#include "AP_Networking_Config.h"

#if AP_NETWORKING_ENABLED

#include "AP_Networking.h"
#include <AP_HAL/utility/Socket.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL& hal;

#ifndef AP_NETWORKING_PORT_MIN_TXSIZE
#define AP_NETWORKING_PORT_MIN_TXSIZE 2048
#endif

#ifndef AP_NETWORKING_PORT_MIN_RXSIZE
#define AP_NETWORKING_PORT_MIN_RXSIZE 2048
#endif

const AP_Param::GroupInfo AP_Networking::Port::var_info[] = {
    // @Param: TYPE
    // @DisplayName: Port type
    // @Description: Port type
    // @Values: 0:Disabled, 1:UDP client
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO_FLAGS("TYPE", 1,  AP_Networking::Port, type, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: PROTOCOL
    // @DisplayName: protocol
    // @Description: protocol
    // @User: Advanced
    // @CopyFieldsFrom: SERIAL0_PROTOCOL
    AP_GROUPINFO("PROTOCOL", 2,  AP_Networking::Port, state.protocol, 0),

    // @Group: IP
    // @Path: AP_Networking_address.cpp
    AP_SUBGROUPINFO(ip, "IP", 3,  AP_Networking::Port, AP_Networking_IPV4),

    // @Param: PORT
    // @DisplayName: Port number
    // @Description: Port number
    // @Range: 0 65535
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("PORT", 4,  AP_Networking::Port, port, 0),
    
    AP_GROUPEND
};

/*
  initialise mapped network ports
 */
void AP_Networking::ports_init(void)
{
    for (uint8_t i=0; i<ARRAY_SIZE(ports); i++) {
        auto &p = ports[i];
        NetworkPortType ptype = (NetworkPortType)p.type;
        p.state.idx = AP_SERIALMANAGER_NET_PORT_1 + i;
        switch (ptype) {
        case NetworkPortType::NONE:
            break;
        case NetworkPortType::UDP_CLIENT:
            p.udp_client_init(AP_NETWORKING_PORT_MIN_RXSIZE, AP_NETWORKING_PORT_MIN_TXSIZE);
            break;
        }
        if (p.sock != nullptr) {
            AP::serialmanager().register_port(&p);
        }
    }
}

/*
  initialise a UDP client
 */
void AP_Networking::Port::udp_client_init(const uint32_t size_rx, const uint32_t size_tx)
{
    WITH_SEMAPHORE(sem);
    if (!init_buffers(size_rx, size_tx)) {
        return;
    }
    if (sock != nullptr) {
        return;
    }
    sock = new SocketAPM(true);
    if (sock == nullptr) {
        return;
    }
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Networking::Port::udp_client_loop, void), "NET", 2048, AP_HAL::Scheduler::PRIORITY_UART, 0)) {
        AP_BoardConfig::allocation_error("Failed to allocate UDP client thread");
    }
}

/*
  update a UDP client
 */
void AP_Networking::Port::udp_client_loop(void)
{
    while (!hal.scheduler->is_system_initialized()) {
        hal.scheduler->delay(100);
    }
    hal.scheduler->delay(1000);

    const char *dest = ip.get_str();
    if (!sock->connect(dest, port.get())) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "UDP[%u]: Failed to connect to %s", (unsigned)state.idx, dest);
        delete sock;
        sock = nullptr;
        return;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "UDP[%u]: connected to %s:%u", (unsigned)state.idx, dest, unsigned(port.get()));

    while (true) {
        hal.scheduler->delay_microseconds(500);
        WITH_SEMAPHORE(sem);

        // handle outgoing packets
        uint32_t available;
        const auto *ptr = writebuffer->readptr(available);
        if (ptr != nullptr) {
            const auto ret = sock->send(ptr, available);
            if (ret > 0) {
                writebuffer->advance(ret);
            }
        }

        // handle incoming packets
        const auto space = readbuffer->space();
        if (space > 0) {
            const uint32_t n = MIN(350U, space);
            uint8_t buf[n];
            const auto ret = sock->recv(buf, n, 0);
            if (ret > 0) {
                readbuffer->write(buf, ret);
            }
        }
    }
}

/*
  available space in outgoing buffer
 */
uint32_t AP_Networking::Port::txspace(void)
{
    WITH_SEMAPHORE(sem);
    return writebuffer->space();
}

void AP_Networking::Port::_begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
    rxS = MAX(rxS, AP_NETWORKING_PORT_MIN_RXSIZE);
    txS = MAX(txS, AP_NETWORKING_PORT_MIN_TXSIZE);
    init_buffers(rxS, txS);
}

size_t AP_Networking::Port::_write(const uint8_t *buffer, size_t size)
{
    WITH_SEMAPHORE(sem);
    return writebuffer->write(buffer, size);
}

ssize_t AP_Networking::Port::_read(uint8_t *buffer, uint16_t count)
{
    WITH_SEMAPHORE(sem);
    return readbuffer->read(buffer, count);
}

uint32_t AP_Networking::Port::_available()
{
    WITH_SEMAPHORE(sem);
    return readbuffer->available();
}


bool AP_Networking::Port::_discard_input()
{
    WITH_SEMAPHORE(sem);
    readbuffer->clear();
    return true;
}

/*
  initialise read/write buffers
 */
bool AP_Networking::Port::init_buffers(const uint32_t size_rx, const uint32_t size_tx)
{
    if (size_tx == last_size_tx &&
        size_rx == last_size_rx) {
        return true;
    }
    WITH_SEMAPHORE(sem);
    if (readbuffer == nullptr) {
        readbuffer = new ByteBuffer(size_rx);
    } else {
        readbuffer->set_size_best(size_rx);
    }
    if (writebuffer == nullptr) {
        writebuffer = new ByteBuffer(size_tx);
    } else {
        writebuffer->set_size_best(size_tx);
    }
    last_size_rx = size_rx;
    last_size_tx = size_tx;
    return readbuffer != nullptr && writebuffer != nullptr;
}

#endif // AP_NETWORKING_ENABLED
