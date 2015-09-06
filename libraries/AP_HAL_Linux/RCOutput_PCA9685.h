
#ifndef __AP_HAL_LINUX_RCOUTPUT_PCA9685_H__
#define __AP_HAL_LINUX_RCOUTPUT_PCA9685_H__

#include "AP_HAL_Linux.h"

#define PCA9685_PRIMARY_ADDRESS             0x40 // All address pins low, PCA9685 default
#define PCA9685_SECONDARY_ADDRESS           0x41
#define PCA9685_TERTIARY_ADDRESS            0x42

class Linux::LinuxRCOutput_PCA9685 : public AP_HAL::RCOutput_Backend {
    public:

    LinuxRCOutput_PCA9685(uint8_t addr);    
    LinuxRCOutput_PCA9685(uint8_t addr, bool external_clock, uint8_t channel_offset,
                          int16_t oe_pin_number);
    ~LinuxRCOutput_PCA9685();
    bool     init();
    uint8_t  get_num_channels();
    void     reset_all_channels();
    void     set_freq(uint64_t chmask, uint16_t freq_hz);
    uint16_t get_freq(uint8_t ch);
    void     enable_ch(uint8_t ch);
    void     disable_ch(uint8_t ch);
    void     write(uint8_t ch, uint16_t period_us);
    uint16_t read(uint8_t ch);

private:
    void reset();

    uint8_t _addr;
    AP_HAL::Semaphore *_i2c_sem;
    AP_HAL::DigitalSource *_enable_pin;
    uint16_t _frequency;
    float _osc_clock;
    uint16_t *_pulses_buffer;
    bool _external_clock;
    uint8_t _channel_offset;
    int16_t _oe_pin_number;
};

#endif // __AP_HAL_LINUX_RCOUTPUT_PCA9685_H__
