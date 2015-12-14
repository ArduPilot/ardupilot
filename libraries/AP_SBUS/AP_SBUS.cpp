#include "AP_SBUS.h"
#include <cstring>
#include <math.h>

#define SBUS_STATE_FAILSAFE         (1 << 0)
#define SBUS_STATE_SIGNALLOSS       (1 << 1)

#define SBUS_FLAG_CHANNEL_17        (1 << 0)
#define SBUS_FLAG_CHANNEL_18        (1 << 1)
#define SBUS_FLAG_SIGNAL_LOSS       (1 << 2)
#define SBUS_FLAG_FAILSAFE_ACTIVE   (1 << 3)

#define SBUS_FRAME_BEGIN_BYTE       0x0F

#define SBUS_DIGITAL_CHANNEL_MIN    173
#define SBUS_DIGITAL_CHANNEL_MAX    1812


extern const AP_HAL::HAL& hal;


AP_SBUS::AP_SBUS() {
    std::memset(_channels, 0, sizeof(_channels));
    _goodFrames  = 0;
    _lostFrames  = 0;
    _errorFrames = 0;
    _state       = 0;
}

bool AP_SBUS::iser_to_sbus(const uint16_t buffer[25], const size_t &bytes) {   
    // byte inverter, and uint16_t to uint8_t byte splitter
    for (int i = 0; i < bytes; i++) {
        _sbus_frame.bytes[i] = (((~buffer[i]) >> 1) & 0xFF);
    }
    
    // check parity byte
    for (uint8_t i = 0; i < bytes; i++) {
        uint8_t parity = 0;
        for (uint8_t j = 1; j <= 8; j++) {
            parity ^= ((~buffer[i]) & (1U << j)) ? 1 : 0;
        }
        
        if (parity != ((~buffer[i]) & 0x200) >> 9) {
            return false;
        }
    }
     
    // sanity check
    if (_sbus_frame.bytes[0] != SBUS_FRAME_BEGIN_BYTE) {
        return false;
    }
    
    return true;
}

void AP_SBUS::process_frame(const uint16_t buffer[25], const size_t &bytes) {
    if(iser_to_sbus(buffer, bytes) == false) {
        _errorFrames++;
        return;
    }
  
    _channels[0] = _sbus_frame.frame.chan0;
    _channels[1] = _sbus_frame.frame.chan1;
    _channels[2] = _sbus_frame.frame.chan2;
    _channels[3] = _sbus_frame.frame.chan3;
    _channels[4] = _sbus_frame.frame.chan4;
    _channels[5] = _sbus_frame.frame.chan5;
    _channels[6] = _sbus_frame.frame.chan6;
    _channels[7] = _sbus_frame.frame.chan7;
    _channels[8] = _sbus_frame.frame.chan8;
    _channels[9] = _sbus_frame.frame.chan9;
    _channels[10] = _sbus_frame.frame.chan10;
    _channels[11] = _sbus_frame.frame.chan11;
    _channels[12] = _sbus_frame.frame.chan12;
    _channels[13] = _sbus_frame.frame.chan13;
    _channels[14] = _sbus_frame.frame.chan14;
    _channels[15] = _sbus_frame.frame.chan15;

    _sbus_frame.frame.flags & SBUS_FLAG_CHANNEL_17 ? _channels[16] = SBUS_DIGITAL_CHANNEL_MAX : _channels[16] = SBUS_DIGITAL_CHANNEL_MIN;
    _sbus_frame.frame.flags & SBUS_FLAG_CHANNEL_18 ? _channels[17] = SBUS_DIGITAL_CHANNEL_MAX : _channels[17] = SBUS_DIGITAL_CHANNEL_MIN;

    _state = 0;
    if (_sbus_frame.frame.flags & SBUS_FLAG_SIGNAL_LOSS) {
        _state |= SBUS_STATE_SIGNALLOSS;
        _lostFrames++;
    }
    
    if (_sbus_frame.frame.flags & SBUS_FLAG_FAILSAFE_ACTIVE) {
        _state |= SBUS_STATE_FAILSAFE;
    }
    
    if(_state == 0) {
        _goodFrames++;
    }
}

uint16_t AP_SBUS::get_channel_dig(const uint8_t &channel) const {
    if (channel < 1 or channel > 18) {
      return 0;
    }
    return _channels[channel-1];
}

uint16_t AP_SBUS::get_channel_pwm(const uint8_t &channel) const {
    if (channel < 1 or channel > 18) {
        return 0;
    }
    // Linear fitting values read from OpenTX-ppmus and comparing with values received by X4R
    // http://www.wolframalpha.com/input/?i=linear+fit+%7B173%2C+988%7D%2C+%7B1812%2C+2012%7D%2C+%7B993%2C+1500%7D
    return (uint16_t)(0.625f * (float)get_channel_dig(channel)) + 880;
}

int16_t AP_SBUS::get_state() const {
    return _state;
}

int16_t AP_SBUS::get_frame_loss() const {
    return (_lostFrames + _errorFrames) * 100 / (_goodFrames + _lostFrames + _errorFrames);
}

uint32_t AP_SBUS::get_good_frames() const {
    return _goodFrames;
}

uint32_t AP_SBUS::get_lost_frames() const {
    return _lostFrames;
}

uint32_t AP_SBUS::get_error_frames() const {
    return _errorFrames;
}

