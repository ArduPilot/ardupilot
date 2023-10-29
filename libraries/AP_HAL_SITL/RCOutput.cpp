#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <SITL/SITL.h>

#include "RCOutput.h"

#define ENABLE_DEBUG 0

#if ENABLE_DEBUG
# include <stdio.h>
# define Debug(fmt, args ...)  do {::printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while (0)
#else
# define Debug(fmt, args ...)
#endif

using namespace HALSITL;

void RCOutput::init() {}

void RCOutput::set_freq(uint32_t chmask, uint16_t freq_hz)
{
    Debug("set_freq(0x%04x, %u)\n", static_cast<uint32_t>(chmask), static_cast<uint32_t>(freq_hz));
    _freq_hz = freq_hz;
}

uint16_t RCOutput::get_freq(uint8_t ch)
{
    return _freq_hz;
}

void RCOutput::enable_ch(uint8_t ch)
{
    if (!(_enable_mask & (1U << ch))) {
        Debug("enable_ch(%u)\n", ch);
    }
    _enable_mask |= (1U << ch);
}

void RCOutput::disable_ch(uint8_t ch)
{
    if (_enable_mask & (1U << ch)) {
        Debug("disable_ch(%u)\n", ch);
    }
    _enable_mask &= ~(1U << ch);
}

void RCOutput::write(uint8_t ch, uint16_t period_us)
{
    if (safety_state == AP_HAL::Util::SAFETY_DISARMED) {
        const auto *board_config = AP_BoardConfig::get_singleton();
        const uint32_t safety_mask = board_config != nullptr? board_config->get_safety_mask() : 0;
        if (!(safety_mask & (1U<<ch))) {
            // implement safety pwm value
            period_us = 0;
        }
    }

    _sitlState->output_ready = true;
    // FIXME: something in sitl is expecting to be able to read and write disabled channels
    if (ch < SITL_NUM_CHANNELS /*&& (_enable_mask & (1U<<ch))*/) {
        if (_corked) {
            _pending[ch] = period_us;
        } else {
            _sitlState->pwm_output[ch] = period_us;
        }
    }
}

uint16_t RCOutput::read(uint8_t ch)
{
    // FIXME: something in sitl is expecting to be able to read and write disabled channels
    if (ch < SITL_NUM_CHANNELS /*&& (_enable_mask & (1U<<ch))*/) {
        return _sitlState->pwm_output[ch];
    }
    return 0;
}

void RCOutput::read(uint16_t* period_us, uint8_t len)
{
    memcpy(period_us, _sitlState->pwm_output, len * sizeof(uint16_t));
}

void RCOutput::cork(void)
{
    if (!_corked) {
        memcpy(_pending, _sitlState->pwm_output, SITL_NUM_CHANNELS * sizeof(uint16_t));
        _corked = true;
    }
}

void RCOutput::push(void)
{
    if (_corked) {
        memcpy(_sitlState->pwm_output, _pending, SITL_NUM_CHANNELS * sizeof(uint16_t));
        _corked = false;
    }

    SITL::SIM *sitl = AP::sitl();
    if (sitl && sitl->esc_telem) {
        if (esc_telem == nullptr) {
            esc_telem = new AP_ESC_Telem_SITL;
        }
        if (esc_telem != nullptr) {
            esc_telem->update();
        }
    }
}

/*
  Serial LED emulation
*/
bool RCOutput::set_serial_led_num_LEDs(const uint16_t chan, uint8_t num_leds, output_mode mode, uint32_t clock_mask)
{
    if (chan > 15 || num_leds > 64) {
        return false;
    }
    SITL::SIM *sitl = AP::sitl();
    if (sitl) {
        sitl->led.num_leds[chan] = num_leds;
        return true;
    }
    return false;
}

void RCOutput::set_serial_led_rgb_data(const uint16_t chan, int8_t led, uint8_t red, uint8_t green, uint8_t blue)
{
    if (chan > 15) {
        return;
    }
    SITL::SIM *sitl = AP::sitl();
    if (led == -1) {
        for (uint8_t i=0; i < sitl->led.num_leds[chan]; i++) {
            set_serial_led_rgb_data(chan, i, red, green, blue);
        }
        return;
    }
    if (led < -1 || led >= sitl->led.num_leds[chan]) {
        return;
    }
    if (sitl) {
        sitl->led.rgb[chan][led].rgb[0] = red;
        sitl->led.rgb[chan][led].rgb[1] = green;
        sitl->led.rgb[chan][led].rgb[2] = blue;
    }
}

void RCOutput::serial_led_send(const uint16_t chan)
{
    SITL::SIM *sitl = AP::sitl();
    if (sitl) {
        sitl->led.send_counter++;
    }
}

#endif //CONFIG_HAL_BOARD == HAL_BOARD_SITL
