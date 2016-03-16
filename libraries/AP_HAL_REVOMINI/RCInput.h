
#ifndef __AP_HAL_REVOMINI_RCINPUT_H__
#define __AP_HAL_REVOMINI_RCINPUT_H__

#include <AP_HAL_REVOMINI/AP_HAL_REVOMINI.h>
#include <AP_HAL/AP_HAL.h>

#define REVOMINI_RC_INPUT_MIN_CHANNELS 4
#define REVOMINI_RC_INPUT_NUM_CHANNELS 8
#define PPM_SUM_CHANNEL 75

class REVOMINI::REVOMINIRCInput : public AP_HAL::RCInput {
public:
    REVOMINIRCInput();
    void init();
    uint8_t  valid_channels();

    uint16_t read(uint8_t ch);
    uint8_t read(uint16_t* periods, uint8_t len);
    
    bool new_input();
    uint8_t num_channels();

    bool set_overrides(int16_t *overrides, uint8_t len);
    bool set_override(uint8_t channel, int16_t override);
    void clear_overrides();
private:
    static void rxIntPPMSUM(uint8_t state, uint16_t value);
    void InitDefaultPPM(char board);

    unsigned int ppm_sum_channel;
    unsigned int input_channel_ch1;
    unsigned int input_channel_ch2;
    unsigned int input_channel_ch3;
    unsigned int input_channel_ch4;
    unsigned int input_channel_ch5;
    unsigned int input_channel_ch6;
    unsigned int input_channel_ch7;
    unsigned int input_channel_ch8;

    unsigned char _iboard;
    static volatile uint8_t  _valid;

    /* override state */
    uint16_t _override[8];

    uint64_t _last_read;
    bool _override_valid;

    static volatile uint64_t _timestamp_last_signal;
    /* private variables to communicate with input capture isr */
    static volatile uint16_t _pulse_capt[REVOMINI_RC_INPUT_NUM_CHANNELS];
    static volatile uint32_t _last_pulse[REVOMINI_RC_INPUT_NUM_CHANNELS];

    static volatile uint8_t  _valid_channels;
};

#endif // __AP_HAL_REVOMINI_RCINPUT_H__
