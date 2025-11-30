#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_IOMCU_ENABLED

#include "AP_RCProtocol_IOMCU.h"

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_IOMCU/AP_IOMCU.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_RCMapper/AP_RCMapper.h>
#include <RC_Channel/RC_Channel.h>

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

    if (rc().option_is_enabled(RC_Channels::Option::PWM_CHECK_VALUE)) {
        // Do not update if the channel PWM is outside the calibration range.
        uint8_t pwm_ch_nums = num_channels > 15 ? num_channels - 2 : num_channels;
        uint8_t thr_ch = AP::rcmap()->throttle() - 1;
        if (!AP_RCProtocol::validate_rc_input_range(rcin_values, num_channels, thr_ch, pwm_ch_nums)) {
            return;
        }
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
