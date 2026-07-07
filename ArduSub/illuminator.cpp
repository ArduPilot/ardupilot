#include "Sub.h"

#include "illuminator.h"

#include <AP_Math/AP_Math.h>

bool AP_Illuminator::set_brightness_pct(float brightness_pct)
{
    if (brightness_pct < 0.0f || brightness_pct > 100.0f) {
        return false;
    }
    _brightness_pct = brightness_pct;
    if (is_positive(_brightness_pct)) {
        _last_on_brightness_pct = _brightness_pct;
    }
    // SRV_Channel outputs are scaled 0-1000.
    SRV_Channels::set_output_scaled(_output_function, _brightness_pct * 10.0f);
    return true;
}

void AP_Illuminator::on_off(bool enable)
{
    // both values are guaranteed in-range, so set_brightness_pct cannot fail.
    IGNORE_RETURN(set_brightness_pct(enable ? _last_on_brightness_pct : 0.0f));
}

void AP_Illuminator::adjust_brightness_pct(float delta_pct)
{
    // clamp here so the value handed to set_brightness_pct is always in-range.
    IGNORE_RETURN(set_brightness_pct(constrain_float(_brightness_pct + delta_pct, 0.0f, 100.0f)));
}

void AP_Illuminator::cycle_brightness_pct(float step_pct)
{
    adjust_brightness_pct(_cycle_increasing ? step_pct : -step_pct);
    if (_brightness_pct >= 100.0f || _brightness_pct <= 0.0f) {
        _cycle_increasing = !_cycle_increasing;
    }
}

#if AP_MAVLINK_MSG_ILLUMINATOR_STATUS_SENDING_ENABLED && defined(MAVLINK_MSG_ID_ILLUMINATOR_STATUS)
void AP_Illuminator::send_status(mavlink_channel_t chan) const
{
    mavlink_msg_illuminator_status_send(
        chan,
        AP_HAL::millis(),                        // uptime_ms
        is_positive(_brightness_pct) ? 1 : 0,    // enable
        1U << ILLUMINATOR_MODE_INTERNAL_CONTROL, // mode_bitmask
        0,                                       // error_status
        ILLUMINATOR_MODE_INTERNAL_CONTROL,       // mode
        _brightness_pct,                         // brightness (%)
        0.0f,                                    // strobe_period (not supported)
        0.0f,                                    // strobe_duty_cycle (not supported)
        0.0f,                                    // temp_c (not measured)
        0.0f,                                    // min_strobe_period (not supported)
        0.0f                                     // max_strobe_period (not supported)
        // TODO: once mavlink/mavlink#2490 lands and the submodule advances,
        // plumb this illuminator's id through to here so receivers can
        // distinguish between the two ArduSub illuminators.
    );
}
#endif

// Return the illuminator addressed by a MAV_CMD_ILLUMINATOR_* id, or nullptr if
// the id does not name a single autopilot-attached illuminator.  id 0 ("all")
// is not a single instance and so returns nullptr; callers iterate 1..count.
AP_Illuminator *Sub::get_illuminator(uint8_t id)
{
    if (id >= 1 && id <= illuminator_count) {
        return &illuminators[id - 1];
    }
    return nullptr;
}
