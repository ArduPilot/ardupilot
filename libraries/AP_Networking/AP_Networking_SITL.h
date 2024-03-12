#pragma once

#include "AP_Networking_Config.h"

#if AP_NETWORKING_BACKEND_SITL
#include "AP_Networking_Backend.h"

class AP_Networking_SITL : public AP_Networking_Backend
{
public:
    using AP_Networking_Backend::AP_Networking_Backend;

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Networking_SITL);

    bool init() override {
        return true;
    }
    void update() override {}
};

#endif // AP_NETWORKING_BACKEND_SITL
