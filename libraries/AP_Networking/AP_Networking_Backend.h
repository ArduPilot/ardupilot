#pragma once

#include "AP_Networking_Config.h"

#if AP_NETWORKING_ENABLED

#include "AP_Networking.h"

class AP_Networking;

class AP_Networking_Backend
{
public:
    friend class AP_Networking;

    AP_Networking_Backend(AP_Networking &_frontend) : frontend(_frontend) {}

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Networking_Backend);

    virtual bool init() = 0;
    virtual void update() {};

protected:
    AP_Networking &frontend;

    struct {
        uint32_t ip;
        uint32_t nm;
        uint32_t gw;
        uint32_t announce_ms;
        uint8_t macaddr[6];
        uint32_t last_change_ms;
    } activeSettings;
};

#endif // AP_NETWORKING_ENABLED
