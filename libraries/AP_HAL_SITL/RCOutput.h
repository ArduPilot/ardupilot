#pragma once

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include "AP_HAL_SITL.h"

class HALSITL::RCOutput : public AP_HAL::RCOutput {
public:
    explicit RCOutput(SITL_State *sitlState): _sitlState(sitlState), _freq_hz(50) {}
    void init() override;
    void set_freq(uint32_t chmask, uint16_t freq_hz) override;
    uint16_t get_freq(uint8_t ch) override;
    void enable_ch(uint8_t ch) override;
    void disable_ch(uint8_t ch) override;
    void write(uint8_t ch, uint16_t period_us) override;
    uint16_t read(uint8_t ch) override;
    void read(uint16_t* period_us, uint8_t len) override;
    void cork(void) override;
    void push(void) override;

    /*
      force the safety switch on, disabling PWM output from the IO board
     */
    bool force_safety_on(void) override;
    /*
      force the safety switch off, enabling PWM output from the IO board
     */
    void force_safety_off(void) override;

    /*
      Serial LED emulation
     */
    bool set_neopixel_num_LEDs(const uint16_t chan, uint8_t num_leds) override;
    void set_neopixel_rgb_data(const uint16_t chan, uint32_t ledmask, uint8_t red, uint8_t green, uint8_t blue) override;
    void neopixel_send(void) override;
    
private:
    SITL_State *_sitlState;
    uint16_t _freq_hz;
    uint16_t _enable_mask;
    bool _corked;
    uint16_t _pending[SITL_NUM_CHANNELS];
};

#endif
