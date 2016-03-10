
#ifndef __AP_HAL_REVOMINI_RCOUTPUT_H__
#define __AP_HAL_REVOMINI_RCOUTPUT_H__

#include <AP_HAL_REVOMINI/AP_HAL_REVOMINI.h>
#include <AP_HAL/AP_HAL.h>
#include <timer.h>

#define REVOMINI_MAX_OUTPUT_CHANNELS 12

#define MOTORID1 0
#define MOTORID2 1
#define MOTORID3 2
#define MOTORID4 3
#define MOTORID5 4
#define MOTORID6 5
#define MOTORID7 6
#define MOTORID8 7
#define MOTORID9 8
#define MOTORID10 9
#define MOTORID11 10
#define MOTORID12 11

class REVOMINI::REVOMINIRCOutput : public AP_HAL::RCOutput {
    void     init();
    void     set_freq(uint32_t chmask, uint16_t freq_hz);
    uint16_t get_freq(uint8_t ch);
    void     enable_ch(uint8_t ch);
    void     enable_mask(uint32_t chmask);
    void     disable_ch(uint8_t ch);
    void     disable_mask(uint32_t chmask);
    void     write(uint8_t ch, uint16_t period_us);
    void     write(uint8_t ch, uint16_t* period_us, uint8_t len);
    uint16_t read(uint8_t ch);
    void     read(uint16_t* period_us, uint8_t len);
private:
    void InitDefaultPWM(void);
    void InitPWM(void);
    uint32_t _timer_period(uint16_t speed_hz);

    unsigned int output_channel_ch1;
    unsigned int output_channel_ch2;
    unsigned int output_channel_ch3;
    unsigned int output_channel_ch4;
    unsigned int output_channel_ch5;
    unsigned int output_channel_ch6;
    unsigned int output_channel_ch7;
    unsigned int output_channel_ch8;
    unsigned int output_channel_ch9;
    unsigned int output_channel_ch10;
    unsigned int output_channel_ch11;
    unsigned int output_channel_ch12;
    unsigned int output_channel_raw[REVOMINI_MAX_OUTPUT_CHANNELS];

};

#endif // __AP_HAL_REVOMINI_RCOUTPUT_H__
