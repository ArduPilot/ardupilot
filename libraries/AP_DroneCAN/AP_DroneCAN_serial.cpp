/*
  map local serial ports to remote DroneCAN serial ports using the
  TUNNEL_TARGETTED message
 */

#include "AP_DroneCAN.h"

#if HAL_ENABLE_DRONECAN_DRIVERS && AP_DRONECAN_SERIAL_ENABLED

#include <AP_Math/AP_Math.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

AP_DroneCAN_Serial *AP_DroneCAN_Serial::serial[HAL_MAX_CAN_PROTOCOL_DRIVERS];

#ifndef AP_DRONECAN_SERIAL_MIN_TXSIZE
#define AP_DRONECAN_SERIAL_MIN_TXSIZE 2048
#endif

#ifndef AP_DRONECAN_SERIAL_MIN_RXSIZE
#define AP_DRONECAN_SERIAL_MIN_RXSIZE 2048
#endif

/*
  initialise DroneCAN serial aports
*/
void AP_DroneCAN_Serial::init(AP_DroneCAN *_dronecan)
{
    if (enable == 0) {
        return;
    }
    const uint8_t driver_index = _dronecan->get_driver_index();
    if (driver_index >= ARRAY_SIZE(serial)) {
        return;
    }
    serial[driver_index] = this;
    dronecan = _dronecan;

    const uint8_t base_port = driver_index == 0? AP_SERIALMANAGER_CAN_D1_PORT_1 : AP_SERIALMANAGER_CAN_D2_PORT_1;
    bool need_subscriber = false;

    for (uint8_t i=0; i<ARRAY_SIZE(ports); i++) {
        auto &p = ports[i];
        p.state.idx = base_port + i;
        if (p.node > 0 && p.idx >= 0) {
            p.init();
            AP::serialmanager().register_port(&p);
            need_subscriber = true;
        }
    }

    if (need_subscriber) {
        if (Canard::allocate_sub_arg_callback(dronecan, &handle_tunnel_targetted, dronecan->get_driver_index()) == nullptr) {
            AP_BoardConfig::allocation_error("serial_tunnel_sub");
        }
        targetted = new Canard::Publisher<uavcan_tunnel_Targetted>(dronecan->get_canard_iface());
        if (targetted == nullptr) {
            AP_BoardConfig::allocation_error("serial_tunnel_pub");
        }
        targetted->set_timeout_ms(20);
        targetted->set_priority(CANARD_TRANSFER_PRIORITY_MEDIUM);
    }
}

/*
  update DroneCAN serial ports
*/
void AP_DroneCAN_Serial::update(void)
{
    const uint32_t now_ms = AP_HAL::millis();
    for (auto &p : ports) {
        if (p.baudrate == 0) {
            continue;
        }
        if (p.writebuffer == nullptr || p.node <= 0 || p.idx < 0) {
            continue;
        }
        uavcan_tunnel_Targetted pkt {};
        uint32_t n = 0;
        {
            WITH_SEMAPHORE(p.sem);
            uint32_t avail;
            const bool send_keepalive = now_ms - p.last_send_ms > 500;
            const auto *ptr = p.writebuffer->readptr(avail);
            if (!send_keepalive && (ptr == nullptr || avail <= 0)) {
                continue;
            }
            n = MIN(avail, sizeof(pkt.buffer.data));
            pkt.target_node = p.node;
            pkt.protocol.protocol = UAVCAN_TUNNEL_PROTOCOL_UNDEFINED;
            pkt.buffer.len = n;
            pkt.baudrate = p.baudrate;
            pkt.serial_id = p.idx;
            pkt.options = UAVCAN_TUNNEL_TARGETTED_OPTION_LOCK_PORT;
            if (ptr != nullptr) {
                memcpy(pkt.buffer.data, ptr, n);
            }
        }
        if (targetted->broadcast(pkt)) {
            WITH_SEMAPHORE(p.sem);
            p.writebuffer->advance(n);
            p.last_send_ms = now_ms;
        }
    }
}

/*
  handle incoming tunnel serial packet
 */
void AP_DroneCAN_Serial::handle_tunnel_targetted(AP_DroneCAN *dronecan,
                                                 const CanardRxTransfer& transfer,
                                                 const uavcan_tunnel_Targetted &msg)
{
    uint8_t driver_index = dronecan->get_driver_index();
    if (driver_index >= ARRAY_SIZE(serial) || serial[driver_index] == nullptr) {
        return;
    }
    auto &s = *serial[driver_index];
    for (auto &p : s.ports) {
        if (p.idx == msg.serial_id && transfer.source_node_id == p.node) {
            WITH_SEMAPHORE(p.sem);
            if (p.readbuffer != nullptr) {
                p.readbuffer->write(msg.buffer.data, msg.buffer.len);
                p.last_recv_us = AP_HAL::micros64();
            }
            break;
        }
    }
}

/*
  initialise port
 */
void AP_DroneCAN_Serial::Port::init(void)
{
    baudrate = state.baud;
    begin(baudrate, 0, 0);
}

/*
  available space in outgoing buffer
 */
uint32_t AP_DroneCAN_Serial::Port::txspace(void)
{
    WITH_SEMAPHORE(sem);
    return writebuffer != nullptr ? writebuffer->space() : 0;
}

void AP_DroneCAN_Serial::Port::_begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
    rxS = MAX(rxS, AP_DRONECAN_SERIAL_MIN_RXSIZE);
    txS = MAX(txS, AP_DRONECAN_SERIAL_MIN_TXSIZE);
    init_buffers(rxS, txS);
    if (b != 0) {
        baudrate = b;
    }
}

size_t AP_DroneCAN_Serial::Port::_write(const uint8_t *buffer, size_t size)
{
    WITH_SEMAPHORE(sem);
    return writebuffer != nullptr ? writebuffer->write(buffer, size) : 0;
}

ssize_t AP_DroneCAN_Serial::Port::_read(uint8_t *buffer, uint16_t count)
{
    WITH_SEMAPHORE(sem);
    return readbuffer != nullptr ? readbuffer->read(buffer, count) : -1;
}

uint32_t AP_DroneCAN_Serial::Port::_available()
{
    WITH_SEMAPHORE(sem);
    return readbuffer != nullptr ? readbuffer->available() : 0;
}


bool AP_DroneCAN_Serial::Port::_discard_input()
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
bool AP_DroneCAN_Serial::Port::init_buffers(const uint32_t size_rx, const uint32_t size_tx)
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

/*
  return timestamp estimate in microseconds for when the start of
  a nbytes packet arrived on the uart.
*/
uint64_t AP_DroneCAN_Serial::Port::receive_time_constraint_us(uint16_t nbytes)
{
    WITH_SEMAPHORE(sem);
    uint64_t last_receive_us = last_recv_us;
    if (baudrate > 0) {
        // assume 10 bits per byte. 
        uint32_t transport_time_us = (1000000UL * 10UL / baudrate) * (nbytes+available());
        last_receive_us -= transport_time_us;
    }
    return last_receive_us;
}

#endif // HAL_ENABLE_DRONECAN_DRIVERS && AP_DRONECAN_SERIAL_ENABLED
