
#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_MAVLINK_RADIO_ENABLED

#include "AP_RCProtocol_MAVLinkRadio.h"

void AP_RCProtocol_MAVLinkRadio::update_radio_rc_channels(const mavlink_radio_rc_channels_t* packet)
{
    const uint8_t count = MIN(packet->count, MAX_RCIN_CHANNELS);

    uint16_t rc_chan[MAX_RCIN_CHANNELS];
    for (uint8_t i = 0; i < count; i++) {
        // The channel values are in centered 13 bit format. Range is [-4096,4096], center is 0.
        // According to specification, the conversion to PWM is x * 5/32 + 1500.
        rc_chan[i] = ((int32_t)packet->channels[i] * 5) / 32 + 1500;
    }

    bool failsafe = (packet->flags & RADIO_RC_CHANNELS_FLAGS_FAILSAFE);

    add_input(count, rc_chan, failsafe);
}

#endif // AP_RCPROTOCOL_MAVLINK_RADIO_ENABLED

