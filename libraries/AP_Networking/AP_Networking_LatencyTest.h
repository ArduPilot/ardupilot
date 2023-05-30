
#pragma once

#include "AP_Networking_Backend.h"

#if AP_NETWORKING_LATENCYTEST_ENABLED

class AP_Networking_LatencyTest  : public AP_Networking_Backend {
public:
    AP_Networking_LatencyTest(AP_Networking &front,
                        AP_Networking::AP_Networking_State &state,
                        AP_Networking_Params &params);

    void init() override;

    void update() override;

    static const struct AP_Param::GroupInfo var_info[];

private:
    
    static void latencytest_recv_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);

    struct {
        struct udp_pcb *pcb;
        AP_Int16 ip[4];
        AP_Int32 port;
    } _eth;

    static constexpr uint32_t MAGIC_VALUE = 0x11223344;
    struct LatencyTestPacket_t {
        uint32_t magic;
        uint32_t data;
    };

    struct {
        uint32_t update_last_ms;
        uint32_t last_rx_data;
        uint32_t last_rx_data_1Hz;
        HAL_Semaphore sem;
    } _stats;

};

#endif // AP_NETWORKING_LATENCYTEST_ENABLED
