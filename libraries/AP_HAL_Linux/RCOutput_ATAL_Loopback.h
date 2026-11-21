#pragma once

#include <AP_HAL/RCOutput.h>

namespace Linux {

// Software-only observer for the generic Linux ArduSub build.
class RCOutput_ATAL_Loopback : public AP_HAL::RCOutput {
public:
    void init() override;
    void set_freq(uint32_t chmask, uint16_t freq_hz) override;
    uint16_t get_freq(uint8_t ch) override;
    void enable_ch(uint8_t ch) override;
    void disable_ch(uint8_t ch) override;
    void write(uint8_t ch, uint16_t period_us) override;
    uint16_t read(uint8_t ch) override;
    void read(uint16_t *period_us, uint8_t len) override;
    void cork() override;
    void push() override;

private:
    static constexpr uint8_t channel_count = 16;
    uint16_t values[channel_count]{};
    uint16_t frequency_hz = 50;
    int socket_fd = -1;
    bool corked = false;
};

}
