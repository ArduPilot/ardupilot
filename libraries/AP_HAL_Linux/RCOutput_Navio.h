
#ifndef __AP_HAL_LINUX_RCOUTPUT_NAVIO_H__
#define __AP_HAL_LINUX_RCOUTPUT_NAVIO_H__

#include "AP_HAL_Linux.h"

class Linux::LinuxRCOutput_Navio : public AP_HAL::RCOutput {
    public:
    LinuxRCOutput_Navio(bool external_clock, uint8_t channel_offset,
                        uint8_t oe_pin_number);
    ~LinuxRCOutput_Navio();
    void     init(void* machtnichts);
    void     reset_all_channels();
    void     set_freq(uint32_t chmask, uint16_t freq_hz);
    uint16_t get_freq(uint8_t ch);
    void     enable_ch(uint8_t ch);
    void     disable_ch(uint8_t ch);
    void     write(uint8_t ch, uint16_t period_us);
    void     write(uint8_t ch, uint16_t* period_us, uint8_t len);
    uint16_t read(uint8_t ch);
    void     read(uint16_t* period_us, uint8_t len);

private:
    void reset();

    AP_HAL::Semaphore *_i2c_sem;
    AP_HAL::DigitalSource *_enable_pin;
    uint16_t _frequency;
    float _osc_clock;

    uint16_t *_pulses_buffer;

    bool _external_clock;
    uint8_t _channel_offset;
    uint8_t _oe_pin_number;
};

#endif // __AP_HAL_LINUX_RCOUTPUT_NAVIO_H__
