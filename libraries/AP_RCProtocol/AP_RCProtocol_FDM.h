#pragma once

#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_FDM_ENABLED

#include "AP_RCProtocol_Backend.h"

class AP_RCProtocol_FDM : public AP_RCProtocol_Backend {
public:

    using AP_RCProtocol_Backend::AP_RCProtocol_Backend;

    void update() override;

    bool active() const { return last_input_ms > 0; }

private:

    uint32_t last_input_ms;
};


#endif  // AP_RCPROTOCOL_FDM_ENABLED
