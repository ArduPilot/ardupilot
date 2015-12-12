#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include "RCInput_UDP.h"
#include <stdio.h>

extern const AP_HAL::HAL& hal;

using namespace Linux;

RCInput_UDP::RCInput_UDP() :
    _port(0),
    _last_buf_ts(0),
    _last_buf_seq(0)
{}

void RCInput_UDP::init()
{
    _port = RCINPUT_UDP_DEF_PORT;
    if(!_socket.bind("0.0.0.0", _port)) {
        hal.console->printf("failed to bind UDP socket\n");
    }

    _socket.set_blocking(false);

    return;
}

void RCInput_UDP::_timer_tick(void)
{
    uint64_t delay;
    uint16_t seq_inc;

    /* Read from udp */
    while (_socket.recv(&_buf, sizeof(_buf), 10) == sizeof(_buf)) {
        if (_buf.version != RCINPUT_UDP_VERSION) {
            hal.console->printf("bad protocol version for UDP RCInput\n");
            return;
        }
        if (_last_buf_ts != 0 &&
            (delay = _buf.timestamp_us - _last_buf_ts) > 100000) {
            hal.console->printf("no rc cmds received for %llu\n", (unsigned long long)delay);
        }
        _last_buf_ts = _buf.timestamp_us;

        if ((seq_inc = _buf.sequence - _last_buf_seq) > 10) {
            hal.console->printf("gap in rc cmds : %u\n", seq_inc);
        }
        _last_buf_seq = _buf.sequence;

        _update_periods(_buf.pwms, RCINPUT_UDP_NUM_CHANNELS);
    }
}
#endif
