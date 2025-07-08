#pragma once

#include "AP_Networking_Config.h"

#ifdef AP_NETWORKING_BACKEND_PPP
#include "AP_Networking_Backend.h"

#ifndef AP_NETWORKING_PPP_NUM_INTERFACES
#define AP_NETWORKING_PPP_NUM_INTERFACES 3
#endif

class AP_Networking_PPP : public AP_Networking_Backend
{
public:
    using AP_Networking_Backend::AP_Networking_Backend;

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Networking_PPP);

    bool init() override;

    // hook for custom routes
    struct netif *routing_hook(uint32_t dest) override;
    
private:
    void ppp_loop(void);

    struct PPP_Instance {
        uint8_t idx;
        AP_Networking_PPP *backend;
        AP_HAL::UARTDriver *uart;
        struct netif *pppif;
        struct ppp_pcb_s *ppp;
        bool need_restart;
        uint32_t last_read_ms;
    } iface[AP_NETWORKING_PPP_NUM_INTERFACES];

    void restart_instance(const uint8_t idx);
    bool update_instance(const uint8_t idx);

    static void ppp_status_callback(struct ppp_pcb_s *pcb, int code, void *ctx);
    static uint32_t ppp_output_cb(struct ppp_pcb_s *pcb, const void *data, uint32_t len, void *ctx);
};

#endif // AP_NETWORKING_BACKEND_PPP
