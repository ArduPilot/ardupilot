#pragma once

#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_JOYSTICK_SFML_ENABLED

#include "AP_RCProtocol_Backend.h"

class AP_RCProtocol_Joystick_SFML : public AP_RCProtocol_Backend {
public:

    AP_RCProtocol_Joystick_SFML(AP_RCProtocol &_frontend) :
        AP_RCProtocol_Backend(_frontend) {
    }

    void update() override;

private:

    uint32_t last_receive_ms;
};


#endif  // AP_RCPROTOCOL_JOYSTICK_SFML_ENABLED
