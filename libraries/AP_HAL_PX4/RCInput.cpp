#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include "RCInput.h"
#include <fcntl.h>
#include <unistd.h>
#include <drivers/drv_pwm_output.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>

#include <GCS_MAVLink/GCS.h>

using namespace PX4;

extern const AP_HAL::HAL& hal;

void PX4RCInput::init()
{
    _perf_rcin = perf_alloc(PC_ELAPSED, "APM_rcin");
    _rc_sub = orb_subscribe(ORB_ID(input_rc));
    if (_rc_sub == -1) {
        AP_HAL::panic("Unable to subscribe to input_rc");
    }
    pthread_mutex_init(&rcin_mutex, nullptr);

#if HAL_RCINPUT_WITH_AP_RADIO
    radio = AP_Radio::instance();
    if (radio) {
        radio->init();
    }
#endif
}

bool PX4RCInput::new_input()
{
    pthread_mutex_lock(&rcin_mutex);
    bool valid = _rcin.timestamp_last_signal != _last_read;
    if (_rcin.rc_failsafe) {
        // don't consider input valid if we are in RC failsafe.
        valid = false;
    }
    _last_read = _rcin.timestamp_last_signal;
    pthread_mutex_unlock(&rcin_mutex);
    if (_rcin.input_source != last_input_source) {
        gcs().send_text(MAV_SEVERITY_DEBUG, "RCInput: decoding %s", input_source_name(_rcin.input_source));
        last_input_source = _rcin.input_source;
    }
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
    if (ch >= MIN(RC_INPUT_MAX_CHANNELS, _rcin.channel_count)) {
        return 0;
    }
    pthread_mutex_lock(&rcin_mutex);
    uint16_t v = _rcin.values[ch];
    pthread_mutex_unlock(&rcin_mutex);

#if HAL_RCINPUT_WITH_AP_RADIO
    if (radio && ch == 0) {
        // hook to allow for update of radio on main thread, for mavlink sends
        radio->update();
    }
#endif

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

const char *PX4RCInput::input_source_name(uint8_t id) const
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


void PX4RCInput::_timer_tick(void)
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

#if HAL_RCINPUT_WITH_AP_RADIO
    if (radio && radio->last_recv_us() != last_radio_us) {
        last_radio_us = radio->last_recv_us();
        pthread_mutex_lock(&rcin_mutex);
        _rcin.timestamp_last_signal = last_radio_us;
        _rcin.channel_count = radio->num_channels();
        for (uint8_t i=0; i<_rcin.channel_count; i++) {
            _rcin.values[i] = radio->read(i);
        }
        pthread_mutex_unlock(&rcin_mutex);
    }
#endif
    
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

#if HAL_RCINPUT_WITH_AP_RADIO
    if (radio) {
        radio->start_recv_bind();
    }
#endif
    
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
