#pragma once

#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS_config.h>
#include <GCS_MAVLink/GCS_MAVLink.h>   // for mavlink_channel_t and MAVLINK_MSG_ID_*

// AP_Illuminator: a single autopilot-attached illuminator (light), driven via
// a SRV_Channel output function (e.g. k_lights1 / k_lights2).
//
// Brightness is expressed as a percentage (0-100%); the underlying SRV output
// is scaled 0-1000.  The class remembers the last positive brightness so it can
// be restored when the illuminator is switched back on.
//
// (Lives in ArduSub/ for now; intended to move into a shared library later.)
class AP_Illuminator {
public:
    explicit AP_Illuminator(SRV_Channel::Function output_function) :
        _output_function(output_function) {}

    // Set the brightness in percent.  Returns false (without changing the
    // output) if brightness_pct is outside 0-100.  A positive brightness is
    // remembered so on_off(true) can restore it.
    bool set_brightness_pct(float brightness_pct) WARN_IF_UNUSED;

    // Current brightness in percent (0-100).
    float get_brightness_pct() const { return _brightness_pct; }

    // Turn the illuminator on (restoring the last positive brightness) or off
    // (remembering the current brightness first so it can be restored).
    void on_off(bool enable);

    // Adjust the brightness by delta_pct, clamped to 0-100%.
    void adjust_brightness_pct(float delta_pct);

    // Step the brightness up or down by step_pct, reversing direction at the
    // 0% / 100% limits (used by the joystick "cycle" button function).
    void cycle_brightness_pct(float step_pct);

#if AP_MAVLINK_MSG_ILLUMINATOR_STATUS_SENDING_ENABLED && defined(MAVLINK_MSG_ID_ILLUMINATOR_STATUS)
    // Emit an ILLUMINATOR_STATUS message describing this illuminator on the
    // given channel.  ArduSub illuminators are brightness-only (internal-control
    // mode, no strobe, no temperature or error reporting).
    void send_status(mavlink_channel_t chan) const;
#endif

private:
    const SRV_Channel::Function _output_function;

    float _brightness_pct;                   // current brightness, 0-100% (zero-init: sub is static)
    float _last_on_brightness_pct = 100.0f;  // restored by on_off(true)
    bool _cycle_increasing = true;           // direction for cycle_brightness_pct
};
