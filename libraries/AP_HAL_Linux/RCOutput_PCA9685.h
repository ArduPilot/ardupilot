#pragma once

#include <AP_HAL/I2CDevice.h>

#include "AP_HAL_Linux.h"

#define PCA9685_PRIMARY_ADDRESS             0x40 // All address pins low, PCA9685 default
#define PCA9685_SECONDARY_ADDRESS           0x41
#define PCA9685_TERTIARY_ADDRESS            0x42
#define PCA9685_QUATENARY_ADDRESS           0x55

namespace Linux {

class RCOutput_PCA9685 : public AP_HAL::RCOutput {
public:
    RCOutput_PCA9685(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                     bool external_clock,
                     uint8_t channel_offset,
                     int16_t oe_pin_number);

    ~RCOutput_PCA9685();
    void     init();
    void     reset_all_channels();
    void     set_freq(uint32_t chmask, uint16_t freq_hz);
    uint16_t get_freq(uint8_t ch);
    void     enable_ch(uint8_t ch);
    void     disable_ch(uint8_t ch);
    void     write(uint8_t ch, uint16_t period_us);
    void     cork() override;
    void     push() override;
    uint16_t read(uint8_t ch);
    void     read(uint16_t* period_us, uint8_t len);

private:
    void reset();

    AP_HAL::DigitalSource *_enable_pin;
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
    uint16_t _frequency;
    float _osc_clock;

    uint16_t *_pulses_buffer;

    bool _external_clock;
    bool _corking = false;
    uint8_t _channel_offset;
    int16_t _oe_pin_number;
    uint16_t _pending_write_mask;
};

}
