#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_IOMCU_ENABLED

#include "AP_RCProtocol_IOMCU.h"

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_IOMCU/AP_IOMCU.h>

extern AP_IOMCU iomcu;

const char *AP_RCProtocol_IOMCU::get_rc_protocol() const
{
    return iomcu.get_rc_protocol();
}

void AP_RCProtocol_IOMCU::update()
{
    if (!AP_BoardConfig::io_enabled()) {
        return;
    }

    uint8_t num_channels;
    uint16_t rcin_values[IOMCU_MAX_RC_CHANNELS] {};

    if (!iomcu.check_rcinput(last_iomcu_us, num_channels, rcin_values, ARRAY_SIZE(rcin_values))) {
        return;
    }
    ever_seen_input = true;
    const int16_t _rssi = iomcu.get_RSSI();

    add_input(
        num_channels,
        rcin_values,
        false,  // failsafe
        _rssi,
        0  // link quality
        );
}

void AP_RCProtocol_IOMCU::start_bind()
{
    if (!AP_BoardConfig::io_enabled()) {
        return;
    }
    iomcu.bind_dsm();
}

#endif // AP_RCPROTOCOL_IOMCU_ENABLED
