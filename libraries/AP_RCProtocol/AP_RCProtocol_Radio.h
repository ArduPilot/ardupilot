#pragma once

#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_RADIO_ENABLED

/*
 * Reads fixed-length packets containing either 8 or 16 2-byte values,
 * and interprets them as RC input.
 */

#include "AP_RCProtocol_Backend.h"

class AP_RCProtocol_Radio : public AP_RCProtocol_Backend {
public:

    using AP_RCProtocol_Backend::AP_RCProtocol_Backend;

    void update() override;

    void start_bind(void) override;

private:

    bool init();
    bool init_done;

    uint32_t last_input_us;
};


#endif  // AP_RCPROTOCOL_RADIO_ENABLED
