
#pragma once

#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_SITL_ENABLED

#include "AP_RCProtocol.h"


class AP_RCProtocol_SITL : public AP_RCProtocol_Backend {
public:

    using AP_RCProtocol_Backend::AP_RCProtocol_Backend;

    void process_bytes(uint8_t *bytes, uint16_t count, uint32_t baudrate) override;
    void update() override;

private:

    uint32_t last_byte_ms;

    // search support:
    uint8_t search_buf[32];
    uint8_t search_buf_ofs;
    uint32_t search_success_correct_size_start_ms;
};

#endif // AP_RCPROTOCOL_MAVLINK_RADIO_ENABLED

