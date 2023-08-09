#pragma once

#include "AP_Networking_Config.h"

#if AP_NETWORKING_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include "AP_Networking_backend.h"

class AP_Networking_ChibiOS : public AP_Networking_backend
{
public:
    using AP_Networking_backend::AP_Networking_backend;

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Networking_ChibiOS);

    bool init() override;
    void update() override;

private:
    bool allocate_buffers(void);
    int32_t send_udp(struct udp_pcb *pcb, const struct ip4_addr &ip4_addr, const uint16_t port, const uint8_t* data, uint16_t data_len);

private:
    struct lwipthread_opts *lwip_options;
    uint8_t macaddr[6];
};

#endif // AP_NETWORKING_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

