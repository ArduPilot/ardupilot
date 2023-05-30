#pragma once

#include "AP_Networking_Config.h"
#if AP_NETWORKING_SERIAL_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>

#include <AP_SerialManager/AP_SerialManager.h>
// #include <lwip/udp.h>
// #include <lwip/ip_addr.h>
#include "AP_Networking.h"

#ifndef AP_NETWORKING_UDP_RX_BUF_SIZE
#define AP_NETWORKING_UDP_RX_BUF_SIZE (AP_NETWORKING_ETHERNET_UDP_PAYLOAD_MAX_SIZE*2)
#endif
#ifndef AP_NETWORKING_UDP_TX_BUF_SIZE
#define AP_NETWORKING_UDP_TX_BUF_SIZE (AP_NETWORKING_ETHERNET_UDP_PAYLOAD_MAX_SIZE*5)
#endif

class AP_Networking_Serial : public AP_HAL::UARTDriver {
public:

    AP_Networking_Serial(AP_SerialManager::SerialExtState::IP_Params &ip_params) :
        _params(ip_params)
    {
        IP_ADDR_FROM_ARRAY(&dst_addr, _params.ip);
    }

    /* Implementations of UARTDriver virtual methods */
    void begin(uint32_t b) override {
        begin(b, 0, 0);
    }
    void begin(uint32_t b, uint16_t rxS, uint16_t txS) override;

    void end() override;
    void flush() override;
    void set_blocking_writes(bool blocking) override;

    bool is_initialized() override {
        return _initialized;
    }

    bool tx_pending() override {
        return _writebuf.available() > 0;
    }

    /* Implementations of Stream virtual methods */
    uint32_t available() override {
        return _readbuf.available();
    }
    uint32_t txspace() override {
        return _writebuf.space();
    }
    bool read(uint8_t &b) override;
    ssize_t read(uint8_t *buffer, uint16_t count) override;

    bool discard_input() override {
        _readbuf.clear();
        return true;
    }

    /* Implementations of Print virtual methods */
    size_t write(uint8_t c) override;
    size_t write(const uint8_t *buffer, size_t size) override;

private:
    bool _initialized = false;

    ip_addr_t dst_addr;
    uint16_t dst_port;

    AP_SerialManager::SerialExtState::IP_Params &_params;

    ByteBuffer _readbuf{0};
    ByteBuffer _writebuf{0};
    bool _passthrough_has_seen_data;

    HAL_Semaphore _rx_sem;
    HAL_Semaphore _tx_sem;

    struct udp_pcb *pcb;
    bool blocking_writes;

    thread_t* volatile thread_ctx;
    char thread_name[10];

    uint8_t ins;
    uint64_t _last_ip_send_us;

    static void udp_recv_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);
    void thread();
};
#endif