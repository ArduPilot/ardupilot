#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include "RCInput.h"
#include <fcntl.h>
#include <unistd.h>
#include <drivers/drv_pwm_output.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>

using namespace PX4;

extern const AP_HAL::HAL& hal;

void PX4RCInput::init()
{
    _perf_rcin = perf_alloc(PC_ELAPSED, "APM_rcin");
    _rc_sub = orb_subscribe(ORB_ID(input_rc));
    if (_rc_sub == -1) {
        AP_HAL::panic("Unable to subscribe to input_rc");
    }
    clear_overrides();
    pthread_mutex_init(&rcin_mutex, nullptr);
}

bool PX4RCInput::new_input()
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
    return valid;
}

uint8_t PX4RCInput::num_channels()
{
    pthread_mutex_lock(&rcin_mutex);
    uint8_t n = _rcin.channel_count;
    pthread_mutex_unlock(&rcin_mutex);
    return n;
}

uint16_t PX4RCInput::read(uint8_t ch)
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

uint8_t PX4RCInput::read(uint16_t* periods, uint8_t len)
{
    if (len > RC_INPUT_MAX_CHANNELS) {
        len = RC_INPUT_MAX_CHANNELS;
    }
    for (uint8_t i = 0; i < len; i++){
        periods[i] = read(i);
    }
    return len;
}

bool PX4RCInput::set_overrides(int16_t *overrides, uint8_t len)
{
    bool res = false;
    for (uint8_t i = 0; i < len; i++) {
        res |= set_override(i, overrides[i]);
    }
    return res;
}

bool PX4RCInput::set_override(uint8_t channel, int16_t override) {
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

void PX4RCInput::clear_overrides()
{
    for (uint8_t i = 0; i < RC_INPUT_MAX_CHANNELS; i++) {
        set_override(i, 0);
    }
}

void PX4RCInput::_timer_tick(void)
{
    perf_begin(_perf_rcin);
    bool rc_updated = false;
    if (orb_check(_rc_sub, &rc_updated) == 0 && rc_updated) {
        pthread_mutex_lock(&rcin_mutex);
        orb_copy(ORB_ID(input_rc), _rc_sub, &_rcin);
        pthread_mutex_unlock(&rcin_mutex);
    }
    // note, we rely on the vehicle code checking new_input()
    // and a timeout for the last valid input to handle failsafe
    perf_end(_perf_rcin);
}

bool PX4RCInput::rc_bind(int dsmMode)
{
    int fd = open("/dev/px4io", 0);
    if (fd == -1) {
        fd = open("/dev/px4fmu", 0);
    }
    if (fd == -1) {
        hal.console->printf("RCInput: failed to open /dev/px4io or /dev/px4fmu\n");
        return false;
    }

    uint32_t mode = (dsmMode == 0) ? DSM2_BIND_PULSES : ((dsmMode == 1) ? DSMX_BIND_PULSES : DSMX8_BIND_PULSES);
    int ret = ioctl(fd, DSM_BIND_START, mode);
    close(fd);
    if (ret != 0) {
        hal.console->printf("RCInput: Unable to start DSM bind\n");
        return false;
    }
    return true;
}

#endif
