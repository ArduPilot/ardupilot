#pragma once

#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_EMLID_RCIO_ENABLED

#include "AP_RCProtocol_Backend.h"

class AP_RCProtocol_Emlid_RCIO : public AP_RCProtocol_Backend {
public:

    using AP_RCProtocol_Backend::AP_RCProtocol_Backend;

    void update() override;

private:
    int open_channel(int channel);

    bool init_done;
    void init();

    uint32_t _last_timestamp;
    static const size_t CHANNEL_COUNT = 16;
    int channels[CHANNEL_COUNT];

};

#endif  // AP_RCPROTOCOL_EMLID_RCIO_ENABLED
