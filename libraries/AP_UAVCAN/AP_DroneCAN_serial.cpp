/*
  map local serial ports to remote DroneCAN serial ports using the
  TUNNEL_TARGETTED message
 */

#include "AP_UAVCAN.h"

#if HAL_ENABLE_LIBUAVCAN_DRIVERS && AP_DRONECAN_SERIAL_ENABLED

#include <AP_Math/AP_Math.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <uavcan/tunnel/Targetted.hpp>

AP_DroneCAN_Serial *AP_DroneCAN_Serial::serial[HAL_MAX_CAN_PROTOCOL_DRIVERS];

#ifndef AP_DRONECAN_SERIAL_MIN_TXSIZE
#define AP_DRONECAN_SERIAL_MIN_TXSIZE 2048
#endif

#ifndef AP_DRONECAN_SERIAL_MIN_RXSIZE
#define AP_DRONECAN_SERIAL_MIN_RXSIZE 2048
#endif

static uavcan::Publisher<uavcan::tunnel::Targetted>* tunnel_targetted[HAL_MAX_CAN_PROTOCOL_DRIVERS];
UC_REGISTRY_BINDER(TunnelTargettedCb, uavcan::tunnel::Targetted);
static uavcan::Subscriber<uavcan::tunnel::Targetted, TunnelTargettedCb> *tunnel_targetted_listener[HAL_MAX_CAN_PROTOCOL_DRIVERS];

/*
  initialise DroneCAN serial aports
*/
void AP_DroneCAN_Serial::init(AP_UAVCAN *_dronecan)
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

    // init in reverse order to keep the linked list in
    // AP_SerialManager in the right order
    for (int8_t i=ARRAY_SIZE(ports)-1; i>= 0; i--) {
        auto &p = ports[i];
        p.state.idx = base_port + i;
        if (p.node > 0) {
            p.init();
            AP::serialmanager().register_port(&p);
            need_subscriber = true;
        }
    }

    if (need_subscriber) {
        tunnel_targetted[driver_index] = new uavcan::Publisher<uavcan::tunnel::Targetted>(*dronecan->get_node());
        tunnel_targetted[driver_index]->setTxTimeout(uavcan::MonotonicDuration::fromMSec(20));
        tunnel_targetted[driver_index]->setPriority(uavcan::TransferPriority::OneHigherThanLowest);

        tunnel_targetted_listener[driver_index] = new uavcan::Subscriber<uavcan::tunnel::Targetted, TunnelTargettedCb>(*dronecan->get_node());
        if (tunnel_targetted_listener[driver_index]) {
            tunnel_targetted_listener[driver_index]->start(TunnelTargettedCb(dronecan, &handle_tunnel_targetted));
        }
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
        if (p.writebuffer == nullptr) {
            continue;
        }
        WITH_SEMAPHORE(p.sem);
        uint32_t avail;
        const bool send_keepalive = now_ms - p.last_send_ms > 500;
        const auto *ptr = p.writebuffer->readptr(avail);
        if (!send_keepalive && (ptr == nullptr || avail <= 0)) {
            continue;
        }
        uavcan::tunnel::Targetted pkt {};
        auto n = MIN(avail, 120U);
        pkt.target_node = p.node;
        pkt.protocol.protocol = 255;
        pkt.baudrate = p.baudrate;
        pkt.serial_id = p.idx;
        pkt.options = 1;
        if (ptr != nullptr) {
            for (uint8_t i=0; i<n; i++) {
                pkt.buffer.push_back(ptr[i]);
            }
        }
        if (tunnel_targetted[dronecan->get_driver_index()]->broadcast(pkt)) {
            p.writebuffer->advance(n);
            p.last_send_ms = now_ms;
        }
    }
}

/*
  handle incoming tunnel serial packet
 */
void AP_DroneCAN_Serial::handle_tunnel_targetted(AP_UAVCAN* dronecan, uint8_t node_id, const TunnelTargettedCb &cb)
{
    uint8_t driver_index = dronecan->get_driver_index();
    if (driver_index >= ARRAY_SIZE(serial) || serial[driver_index] == nullptr) {
        return;
    }
    auto &msg = *cb.msg;
    auto &s = *serial[driver_index];
    for (auto &p : s.ports) {
        if (p.idx == msg.serial_id && node_id == p.node) {
            WITH_SEMAPHORE(p.sem);
            if (p.readbuffer != nullptr) {
                uint8_t buf[120];
                uavcan::copy(msg.buffer.begin(),
                             msg.buffer.end(),
                             buf);
                p.readbuffer->write(buf, msg.buffer.size());
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

void AP_DroneCAN_Serial::Port::begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
    rxS = MAX(rxS, AP_DRONECAN_SERIAL_MIN_RXSIZE);
    txS = MAX(txS, AP_DRONECAN_SERIAL_MIN_TXSIZE);
    init_buffers(rxS, txS);
    if (b != 0) {
        baudrate = b;
    }
}

size_t AP_DroneCAN_Serial::Port::write(uint8_t c)
{
    return write(&c, 1);
}

size_t AP_DroneCAN_Serial::Port::write(const uint8_t *buffer, size_t size)
{
    WITH_SEMAPHORE(sem);
    return writebuffer != nullptr ? writebuffer->write(buffer, size) : 0;
}

int16_t AP_DroneCAN_Serial::Port::read(void)
{
    uint8_t c;
    if (read(&c, 1) == 0) {
        return -1;
    }
    return c;
}

ssize_t AP_DroneCAN_Serial::Port::read(uint8_t *buffer, uint16_t count)
{
    WITH_SEMAPHORE(sem);
    return readbuffer != nullptr ? readbuffer->read(buffer, count) : -1;
}

uint32_t AP_DroneCAN_Serial::Port::available()
{
    WITH_SEMAPHORE(sem);
    return readbuffer != nullptr ? readbuffer->available() : 0;
}


bool AP_DroneCAN_Serial::Port::discard_input()
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
        readbuffer->set_size(size_rx);
    }
    if (writebuffer == nullptr) {
        writebuffer = new ByteBuffer(size_tx);
    } else {
        writebuffer->set_size(size_tx);
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
