#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
#include "RCInput.h"
#include <fcntl.h>
#include <unistd.h>
#include <drivers/drv_pwm_output.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>

#include <GCS_MAVLink/GCS.h>

using namespace VRBRAIN;

extern const AP_HAL::HAL& hal;

void VRBRAINRCInput::init()
{
    _perf_rcin = perf_alloc(PC_ELAPSED, "APM_rcin");
    _rc_sub = orb_subscribe(ORB_ID(input_rc));
    if (_rc_sub == -1) {
        AP_HAL::panic("Unable to subscribe to input_rc");
    }
    clear_overrides();
    pthread_mutex_init(&rcin_mutex, nullptr);
}

bool VRBRAINRCInput::new_input() 
{
    pthread_mutex_lock(&rcin_mutex);
    bool valid = _rcin.timestamp_last_signal != _last_read;
    if (_rcin.rc_failsafe) {
        // don't consider input valid if we are in RC failsafe.
        valid = false;
    }
    if (_override_valid) {
        // if we have RC overrides active, then always consider it valid
        valid = true;
    }
    _last_read = _rcin.timestamp_last_signal;
    _override_valid = false;
    pthread_mutex_unlock(&rcin_mutex);
    if (_rcin.input_source != last_input_source) {
        gcs().send_text(MAV_SEVERITY_DEBUG, "RCInput: decoding %s", input_source_name(_rcin.input_source));
        last_input_source = _rcin.input_source;
    }
    return valid;
}

uint8_t VRBRAINRCInput::num_channels() 
{
    pthread_mutex_lock(&rcin_mutex);
    uint8_t n = _rcin.channel_count;
    pthread_mutex_unlock(&rcin_mutex);
    return n;
}

uint16_t VRBRAINRCInput::read(uint8_t ch)
{
    if (ch >= RC_INPUT_MAX_CHANNELS) {
        return 0;
    }
    pthread_mutex_lock(&rcin_mutex);
    if (_override[ch]) {
        uint16_t v = _override[ch];
        pthread_mutex_unlock(&rcin_mutex);
        return v;
    }
    if (ch >= _rcin.channel_count) {
        pthread_mutex_unlock(&rcin_mutex);
        return 0;
    }
    uint16_t v = _rcin.values[ch];
    pthread_mutex_unlock(&rcin_mutex);
    return v;
}

uint8_t VRBRAINRCInput::read(uint16_t* periods, uint8_t len)
{
    if (len > RC_INPUT_MAX_CHANNELS) {
        len = RC_INPUT_MAX_CHANNELS;
    }
    for (uint8_t i = 0; i < len; i++){
        periods[i] = read(i);
    }
    return len;
}

bool VRBRAINRCInput::set_overrides(int16_t *overrides, uint8_t len)
{
    bool res = false;
    for (uint8_t i = 0; i < len; i++) {
        res |= set_override(i, overrides[i]);
    }
    return res;
}

bool VRBRAINRCInput::set_override(uint8_t channel, int16_t override) {
    if (override < 0) {
        return false; /* -1: no change. */
    }
    if (channel >= RC_INPUT_MAX_CHANNELS) {
        return false;
    }
    _override[channel] = override;
    if (override != 0) {
        _override_valid = true;
        return true;
    }
    return false;
}

void VRBRAINRCInput::clear_overrides()
{
    for (uint8_t i = 0; i < RC_INPUT_MAX_CHANNELS; i++) {
        set_override(i, 0);
    }
}

const char *VRBRAINRCInput::input_source_name(uint8_t id) const
{
    switch(id) {
    case input_rc_s::RC_INPUT_SOURCE_UNKNOWN:         return "UNKNOWN";
    case input_rc_s::RC_INPUT_SOURCE_PX4FMU_PPM:      return "PX4FMU_PPM";
    case input_rc_s::RC_INPUT_SOURCE_PX4IO_PPM:       return "PX4IO_PPM";
    case input_rc_s::RC_INPUT_SOURCE_PX4IO_SPEKTRUM:  return "PX4IO_SPEKTRUM";
    case input_rc_s::RC_INPUT_SOURCE_PX4IO_SBUS:      return "PX4IO_SBUS";
    case input_rc_s::RC_INPUT_SOURCE_PX4IO_ST24:      return "PX4IO_ST24";
    case input_rc_s::RC_INPUT_SOURCE_MAVLINK:         return "MAVLINK";
    case input_rc_s::RC_INPUT_SOURCE_QURT:            return "QURT";
    case input_rc_s::RC_INPUT_SOURCE_PX4FMU_SPEKTRUM: return "PX4FMU_SPEKTRUM";
    case input_rc_s::RC_INPUT_SOURCE_PX4FMU_SBUS:     return "PX4FMU_SBUS";
    case input_rc_s::RC_INPUT_SOURCE_PX4FMU_ST24:     return "PX4FMU_ST24";
    case input_rc_s::RC_INPUT_SOURCE_PX4FMU_SUMD:     return "PX4FMU_SUMD";
    case input_rc_s::RC_INPUT_SOURCE_PX4FMU_DSM:      return "PX4FMU_DSM";
    case input_rc_s::RC_INPUT_SOURCE_PX4IO_SUMD:      return "PX4IO_SUMD";
    case input_rc_s::RC_INPUT_SOURCE_PX4FMU_SRXL:     return "PX4FMU_SRXL";
    case input_rc_s::RC_INPUT_SOURCE_PX4IO_SRXL:      return "PX4IO_SRXL";
    default:                                          return "ERROR";
    }
}


void VRBRAINRCInput::_timer_tick(void)
{
    perf_begin(_perf_rcin);
    bool rc_updated = false;
    if (orb_check(_rc_sub, &rc_updated) == 0 && rc_updated) {
        pthread_mutex_lock(&rcin_mutex);
        orb_copy(ORB_ID(input_rc), _rc_sub, &_rcin);
        if (_rcin.rssi != 0 || _rssi != -1) {
            // always zero means not supported
            _rssi = _rcin.rssi;
        }
        pthread_mutex_unlock(&rcin_mutex);
    }
    // note, we rely on the vehicle code checking new_input()
    // and a timeout for the last valid input to handle failsafe
    perf_end(_perf_rcin);
}

bool VRBRAINRCInput::rc_bind(int dsmMode)
{
















    return true;
}

#endif
