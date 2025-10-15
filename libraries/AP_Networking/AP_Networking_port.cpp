/*
  class for networking mapped ports
 */

#include "AP_Networking_Config.h"

#if AP_NETWORKING_REGISTER_PORT_ENABLED

#include "AP_Networking.h"
#include <AP_HAL/utility/Socket.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_HAL/utility/packetise.h>
#include <errno.h>

extern const AP_HAL::HAL& hal;

#ifndef AP_NETWORKING_PORT_MIN_TXSIZE
#define AP_NETWORKING_PORT_MIN_TXSIZE 2048
#endif

#ifndef AP_NETWORKING_PORT_MIN_RXSIZE
#define AP_NETWORKING_PORT_MIN_RXSIZE 2048
#endif

#ifndef AP_NETWORKING_PORT_STACK_SIZE
#define AP_NETWORKING_PORT_STACK_SIZE 1300
#endif

const AP_Param::GroupInfo AP_Networking::Port::var_info[] = {
    // @Param: TYPE
    // @DisplayName: Port type
    // @Description: Port type for network serial port. For the two client types a valid destination IP address must be set. For the two server types either 0.0.0.0 or a local address can be used. The UDP client type will use broadcast if the IP is set to 255.255.255.255 and will use UDP multicast if the IP is in the multicast address range.
    // @Values: 0:Disabled, 1:UDP client, 2:UDP server, 3:TCP client, 4:TCP server
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO_FLAGS("TYPE", 1,  AP_Networking::Port, type_param, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: PROTOCOL
    // @DisplayName: Protocol
    // @Description: Networked serial port protocol
    // @User: Advanced
    // @RebootRequired: True
    // @CopyFieldsFrom: SERIAL1_PROTOCOL
    AP_GROUPINFO("PROTOCOL", 2,  AP_Networking::Port, state.protocol, 0),

    // @Group: IP
    // @Path: AP_Networking_address.cpp
    // @RebootRequired : True
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
        p.type = (NetworkPortType)p.type_param;
        p.state.idx = AP_SERIALMANAGER_NET_PORT_1 + i;
        switch (p.type) {
        case NetworkPortType::NONE:
            break;
        case NetworkPortType::UDP_CLIENT:
            p.udp_client_init();
            break;
        case NetworkPortType::UDP_SERVER:
            p.udp_server_init();
            break;
        case NetworkPortType::TCP_SERVER:
            p.tcp_server_init();
            break;
        case NetworkPortType::TCP_CLIENT:
            p.tcp_client_init();
            break;
        }
        if (p.sock != nullptr || p.listen_sock != nullptr) {
            AP::serialmanager().register_port(&p);
        }
    }
}

/*
  wrapper for thread_create for port functions
 */
void AP_Networking::Port::thread_create(AP_HAL::MemberProc proc)
{
    const uint8_t idx = state.idx - AP_SERIALMANAGER_NET_PORT_1;
    hal.util->snprintf(thread_name, sizeof(thread_name), "NET_P%u", unsigned(idx));

    if (!init_buffers(AP_NETWORKING_PORT_MIN_RXSIZE, AP_NETWORKING_PORT_MIN_TXSIZE)) {
        AP_BoardConfig::allocation_error("Failed to allocate %s buffers", thread_name);
        return;
    }

    if (!hal.scheduler->thread_create(proc, thread_name, AP_NETWORKING_PORT_STACK_SIZE, AP_HAL::Scheduler::PRIORITY_UART, 0)) {
        AP_BoardConfig::allocation_error("Failed to allocate %s client thread", thread_name);
    }
}

/*
  initialise a UDP client
 */
void AP_Networking::Port::udp_client_init(void)
{
    sock = NEW_NOTHROW SocketAPM(true);
    if (sock == nullptr) {
        return;
    }
    sock->set_blocking(false);

    // setup for packet boundaries if this is mavlink
    packetise = (state.protocol == AP_SerialManager::SerialProtocol_MAVLink ||
                 state.protocol == AP_SerialManager::SerialProtocol_MAVLink2);

    thread_create(FUNCTOR_BIND_MEMBER(&AP_Networking::Port::udp_client_loop, void));
}

/*
  initialise a UDP server
 */
void AP_Networking::Port::udp_server_init(void)
{
    sock = NEW_NOTHROW SocketAPM(true);
    if (sock == nullptr) {
        return;
    }
    sock->set_blocking(false);

    // setup for packet boundaries if this is mavlink
    packetise = (state.protocol == AP_SerialManager::SerialProtocol_MAVLink ||
                 state.protocol == AP_SerialManager::SerialProtocol_MAVLink2);

    thread_create(FUNCTOR_BIND_MEMBER(&AP_Networking::Port::udp_server_loop, void));
}

/*
  initialise a TCP server
 */
void AP_Networking::Port::tcp_server_init(void)
{
    listen_sock = NEW_NOTHROW SocketAPM(false);
    if (listen_sock == nullptr) {
        return;
    }
    listen_sock->reuseaddress();

    thread_create(FUNCTOR_BIND_MEMBER(&AP_Networking::Port::tcp_server_loop, void));
}

/*
  initialise a TCP client
 */
void AP_Networking::Port::tcp_client_init(void)
{
    sock = NEW_NOTHROW SocketAPM(false);
    if (sock != nullptr) {
        sock->set_blocking(true);
        thread_create(FUNCTOR_BIND_MEMBER(&AP_Networking::Port::tcp_client_loop, void));
    }
}

/*
  update a UDP client
 */
void AP_Networking::Port::udp_client_loop(void)
{
    AP::network().startup_wait();

    const char *dest = ip.get_str();
    if (!sock->connect(dest, port.get())) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "UDP[%u]: Failed to connect to %s", (unsigned)state.idx, dest);
        delete sock;
        sock = nullptr;
        return;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "UDP[%u]: connected to %s:%u", (unsigned)state.idx, dest, unsigned(port.get()));

    connected = true;

    bool active = false;
    while (true) {
        if (!active) {
            hal.scheduler->delay_microseconds(100);
        }
        active = send_receive();
    }
}

/*
  update a UDP server
 */
void AP_Networking::Port::udp_server_loop(void)
{
    AP::network().startup_wait();

    const char *addr = ip.get_str();
    if (!sock->bind(addr, port.get())) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "UDP[%u]: Failed to bind to %s:%u", (unsigned)state.idx, addr, unsigned(port.get()));
        delete sock;
        sock = nullptr;
        return;
    }
    sock->reuseaddress();

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "UDP[%u]: bound to %s:%u", (unsigned)state.idx, addr, unsigned(port.get()));

    bool active = false;
    while (true) {
        if (!active) {
            hal.scheduler->delay_microseconds(100);
        }
        active = send_receive();
    }
}

/*
  update a TCP server
 */
void AP_Networking::Port::tcp_server_loop(void)
{
    AP::network().startup_wait();

    const char *addr = ip.get_str();
    if (!listen_sock->bind(addr, port.get()) || !listen_sock->listen(1)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "TCP[%u]: Failed to bind to %s:%u", (unsigned)state.idx, addr, unsigned(port.get()));
        delete listen_sock;
        listen_sock = nullptr;
        return;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TCP[%u]: bound to %s:%u", (unsigned)state.idx, addr, unsigned(port.get()));

    close_on_recv_error = true;

    bool active = false;
    while (true) {
        if (!active) {
            hal.scheduler->delay_microseconds(100);
        }
        if (sock == nullptr) {
            sock = listen_sock->accept(100);
            if (sock != nullptr) {
                sock->set_blocking(false);
                char buf[IP4_STR_LEN];
                uint16_t last_port;
                const char *last_addr = listen_sock->last_recv_address(buf, sizeof(buf), last_port);
                if (last_addr != nullptr) {
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TCP[%u]: connection from %s:%u", (unsigned)state.idx, last_addr, unsigned(last_port));
                }
                connected = true;
                sock->reuseaddress();
            }
        }
        if (sock != nullptr) {
            active = send_receive();
        }
    }
}

/*
  update a TCP client
 */
void AP_Networking::Port::tcp_client_loop(void)
{
    AP::network().startup_wait();

    close_on_recv_error = true;

    bool active = false;
    while (true) {
        if (!active) {
            hal.scheduler->delay_microseconds(100);
        }
        if (sock == nullptr) {
            sock = NEW_NOTHROW SocketAPM(false);
            if (sock == nullptr) {
                continue;
            }
            sock->set_blocking(true);
            connected = false;
        }
        if (!connected) {
            const char *dest = ip.get_str();
            connected = sock->connect_timeout(dest, port.get(), 3000);
            if (connected) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TCP[%u]: connected to %s:%u", unsigned(state.idx), dest, unsigned(port.get()));
                sock->set_blocking(false);
            } else {
                delete sock;
                sock = nullptr;
                // don't try and connect too fast
                hal.scheduler->delay(100);
            }
        }
        if (sock != nullptr && connected) {
            active = send_receive();
        }
    }
}

/*
  run one send/receive loop
 */
bool AP_Networking::Port::send_receive(void)
{

    bool active = false;
    uint32_t space;


    // handle incoming packets
    {
        WITH_SEMAPHORE(sem);
        space = readbuffer->space();
    }
    if (space > 0) {
        const uint32_t n = MIN(300U, space);
        uint8_t buf[n];
        const auto ret = sock->recv(buf, n, 0);
        if (close_on_recv_error && ret == 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TCP[%u]: closed connection", unsigned(state.idx));
            delete sock;
            sock = nullptr;
            return false;
        }
        if (ret > 0) {
            WITH_SEMAPHORE(sem);
            readbuffer->write(buf, ret);

            // Cant track dropped read packets because we only read in what there is space for
            // The socket buffer becomes full and data is lost there
            rx_stats_bytes += ret;

            active = true;
            have_received = true;
        }
    }

    if (type == NetworkPortType::UDP_SERVER && have_received) {
        // connect the socket to the last receive address if we have one
        uint32_t last_addr = 0;
        uint16_t last_port = 0;
        if (sock->last_recv_address(last_addr, last_port)) {
            // we might be disconnected and want to reconnect to a different address/port
            // if we haven't received anything for a while
            bool maybe_disconnected = (AP_HAL::millis() - last_udp_srv_recv_time_ms) > 3000 &&
                                      ((last_addr != last_udp_connect_address) || (last_port != last_udp_connect_port));
            if (maybe_disconnected || !connected) {
                char last_addr_str[IP4_STR_LEN];
                sock->inet_addr_to_str(last_addr, last_addr_str, sizeof(last_addr_str));
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "UDP[%u]: connected to %s:%u", unsigned(state.idx), last_addr_str, unsigned(last_port));
                connected = true;
                last_udp_connect_address = last_addr;
                last_udp_connect_port = last_port;
            }
            // if we received something from the same address, reset the timer
            if (((last_addr == last_udp_connect_address) && (last_port == last_udp_connect_port))) {
                last_udp_srv_recv_time_ms = AP_HAL::millis();
            }
        }
    }

    if (connected) {
        // handle outgoing packets
        uint32_t available;

        {
            WITH_SEMAPHORE(sem);
            available = writebuffer->available();
            available = MIN(300U, available);
#if AP_MAVLINK_PACKETISE_ENABLED
            if (packetise) {
                available = mavlink_packetise(*writebuffer, available);
            }
#endif
            if (available == 0) {
                return active;
            }
        }
        uint8_t buf[available];
        uint32_t n;
        {
            WITH_SEMAPHORE(sem);
            n = writebuffer->peekbytes(buf, available);
        }

        // nothing to send return
        if (n <= 0) {
            return active;
        }

        ssize_t ret = -1;
        if (type == NetworkPortType::UDP_SERVER) {
            // UDP Server uses sendto, allowing us to change the destination address port on the fly
            if(last_udp_connect_address != 0 && last_udp_connect_port != 0) {
                ret = sock->sendto(buf, n, last_udp_connect_address, last_udp_connect_port);
            }
        } else {
            // TCP Server and Client and UDP Client use send
            ret = sock->send(buf, n);
        }

        if (ret > 0) {
            WITH_SEMAPHORE(sem);
            writebuffer->advance(ret);
            tx_stats_bytes += ret;
            active = true;
        } else if (errno == ENOTCONN &&
            (type == NetworkPortType::TCP_CLIENT || type == NetworkPortType::TCP_SERVER)) {
            // close socket and mark as disconnected, so we can reconnect with another client or when server comes back
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TCP[%u]: disconnected", unsigned(state.idx));
            sock->close();
            delete sock;
            sock = nullptr;
            connected = false;
        }
    }

    return active;
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

/*
  return flow control state
 */
enum AP_HAL::UARTDriver::flow_control AP_Networking::Port::get_flow_control(void)
{
    const NetworkPortType ptype = (NetworkPortType)type;
    switch (ptype) {
    case NetworkPortType::TCP_CLIENT:
    case NetworkPortType::TCP_SERVER:
        return AP_HAL::UARTDriver::FLOW_CONTROL_ENABLE;
    case NetworkPortType::UDP_CLIENT:
    case NetworkPortType::UDP_SERVER:
    case NetworkPortType::NONE:
        break;
    }
    return AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE;
}

#endif // AP_NETWORKING_REGISTER_PORT_ENABLED


