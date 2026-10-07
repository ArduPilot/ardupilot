
#pragma once

#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_MAVLINK_RADIO_ENABLED

#include "AP_RCProtocol.h"


class AP_RCProtocol_MAVLinkRadio : public AP_RCProtocol_Backend {
public:

    using AP_RCProtocol_Backend::AP_RCProtocol_Backend;

    // update from mavlink messages
    void update_radio_rc_channels(const mavlink_radio_rc_channels_t* packet) override;

private:
    const uint8_t MAX_CHANNELS = MIN((uint8_t)MAVLINK_MSG_RADIO_RC_CHANNELS_FIELD_CHANNELS_LEN, (uint8_t)MAX_RCIN_CHANNELS);

    uint16_t _channels[MAVLINK_MSG_RADIO_RC_CHANNELS_FIELD_CHANNELS_LEN];
};

#endif // AP_RCPROTOCOL_MAVLINK_RADIO_ENABLED

