#pragma once

#include "AP_Networking_Config.h"

#ifdef AP_NETWORKING_BACKEND_SLIP
#include "AP_Networking_Backend.h"

class AP_Networking_SLIP : public AP_Networking_Backend
{
public:
    using AP_Networking_Backend::AP_Networking_Backend;

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Networking_SLIP);

    bool init() override;

    static AP_HAL::UARTDriver *uart;
    
private:
    void slip_loop(void);

    struct netif *slipif;
    uint8_t num_slip;
};

#endif // AP_NETWORKING_BACKEND_SLIP
