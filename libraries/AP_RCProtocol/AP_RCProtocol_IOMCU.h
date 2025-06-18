#pragma once

#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_IOMCU_ENABLED

#include "AP_RCProtocol_Backend.h"

class AP_RCProtocol_IOMCU : public AP_RCProtocol_Backend {
public:

    using AP_RCProtocol_Backend::AP_RCProtocol_Backend;

    void update() override;

    void start_bind() override;

    bool active() const {
        return ever_seen_input && AP_HAL::micros() - last_iomcu_us < 400000;
    }

    const char *get_rc_protocol() const;

private:

    uint32_t last_iomcu_us;
    bool ever_seen_input;

};


#endif  // AP_RCPROTOCOL_IOMCU_ENABLED
