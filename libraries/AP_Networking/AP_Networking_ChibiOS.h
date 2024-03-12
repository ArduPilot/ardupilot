#pragma once

#include "AP_Networking_Config.h"

#if AP_NETWORKING_BACKEND_CHIBIOS
#include "AP_Networking_Backend.h"

class AP_Networking_ChibiOS : public AP_Networking_Backend
{
public:
    using AP_Networking_Backend::AP_Networking_Backend;

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Networking_ChibiOS);

    bool init() override;
    void update() override;

private:
    bool allocate_buffers(void);

private:
    struct lwipthread_opts *lwip_options;
    uint8_t macaddr[6];
};

#endif // AP_NETWORKING_BACKEND_CHIBIOS

