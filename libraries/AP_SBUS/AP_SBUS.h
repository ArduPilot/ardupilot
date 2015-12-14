#ifndef SBUS_h
#define SBUS_h

#include <AP_HAL/AP_HAL.h>

#define SBUS_MAX_CHANNEL            18
#define SBUS_FRAME_SIZE             25


struct sbusFrame_s {
    uint8_t syncByte;
    // 176 bits of data (11 bits per channel * 16 channels) = 22 bytes.
    unsigned int chan0 : 11;
    unsigned int chan1 : 11;
    unsigned int chan2 : 11;
    unsigned int chan3 : 11;
    unsigned int chan4 : 11;
    unsigned int chan5 : 11;
    unsigned int chan6 : 11;
    unsigned int chan7 : 11;
    unsigned int chan8 : 11;
    unsigned int chan9 : 11;
    unsigned int chan10 : 11;
    unsigned int chan11 : 11;
    unsigned int chan12 : 11;
    unsigned int chan13 : 11;
    unsigned int chan14 : 11;
    unsigned int chan15 : 11;
    uint8_t flags;
    /**
     * The endByte is 0x00 on FrSky and some futaba RX's, on Some SBUS2 RX's the value indicates the telemetry byte that is sent after every 4th sbus frame.
     * See https://github.com/cleanflight/cleanflight/issues/590#issuecomment-101027349 and https://github.com/cleanflight/cleanflight/issues/590#issuecomment-101706023
     */
    uint8_t endByte;
} __attribute__ ((__packed__));

typedef union {
    uint8_t     bytes[SBUS_FRAME_SIZE];
    sbusFrame_s frame;
} sbusFrame_t;


class AP_SBUS {
public:
    AP_SBUS();

    // Here you define the input for the S-BUS parser
    void process_frame(const uint16_t buffer[25], const size_t &bytes);
    
    // Here you can read from the channel
    uint16_t get_channel_dig(const uint8_t &channel) const;
    uint16_t get_channel_pwm(const uint8_t &channel) const;
    
    int16_t get_state() const;
    int16_t get_frame_loss() const;
    uint32_t get_good_frames() const;
    uint32_t get_lost_frames() const;
    uint32_t get_error_frames() const;
    
protected:
    /*
     * Serial bus inverter for creating a valid sbus packet
     */
    bool iser_to_sbus(const uint16_t buffer[25], const size_t &bytes);
    
private:
    sbusFrame_t _sbus_frame;
    uint16_t    _channels[18];
    uint16_t    _state;
        
    uint32_t    _goodFrames;
    uint32_t    _lostFrames;
    uint32_t    _errorFrames;
    uint32_t    _lastGoodFrame;
};

#endif
