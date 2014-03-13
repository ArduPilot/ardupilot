/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  Flymaple port by Mike McCauley
 */
#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_FLYMAPLE

// Flymaple RC Outputs
// Derived from libmaple Servo.cpp

#include "RCOutput.h"
#include "FlymapleWirish.h"

using namespace AP_HAL_FLYMAPLE_NS;

extern const AP_HAL::HAL& hal;

#define MAX_OVERFLOW    ((1 << 16) - 1)

void FLYMAPLERCOutput::init(void* machtnichts) {}

void FLYMAPLERCOutput::set_freq(uint32_t chmask, uint16_t freq_hz) 
{
    for (int i = 0; i < 32; i++) {
        if ((chmask >> i) & 1) {
            _set_freq(i, freq_hz);
        }
    }
}

uint16_t FLYMAPLERCOutput::get_freq(uint8_t ch) 
{
    if (ch >= FLYMAPLE_RC_OUTPUT_NUM_CHANNELS)
	return 0;
    uint8_t pin = _channel_to_flymaple_pin(ch);
    timer_dev *tdev = PIN_MAP[pin].timer_device;

    if (tdev == NULL)
        return 0; // Should never happen
    uint16 prescaler = timer_get_prescaler(tdev);
    uint16 overflow = timer_get_reload(tdev);
    return F_CPU / (prescaler+1) / overflow;
}

void FLYMAPLERCOutput::enable_ch(uint8_t ch)
{
    if (ch >= FLYMAPLE_RC_OUTPUT_NUM_CHANNELS)
	return;
    uint8_t pin = _channel_to_flymaple_pin(ch);
    timer_dev *tdev = PIN_MAP[pin].timer_device;

    if (tdev == NULL) {
        // don't reset any fields or ASSERT(0), to keep driving any
        // previously attach()ed servo.
        return;
    }
    pinMode(pin, PWM);
    _set_freq(ch, 50); // Default to 50 Hz
}

void FLYMAPLERCOutput::disable_ch(uint8_t ch)
{
    if (ch >= FLYMAPLE_RC_OUTPUT_NUM_CHANNELS)
	return;
    uint8_t pin = _channel_to_flymaple_pin(ch);
    timer_dev *tdev = PIN_MAP[pin].timer_device;

    if (tdev == NULL) {
        // don't reset any fields or ASSERT(0), to keep driving any
        // previously attach()ed servo.
        return;
    }
    pinMode(pin, INPUT);
}

void FLYMAPLERCOutput::write(uint8_t ch, uint16_t period_us)
{
    if (ch >= FLYMAPLE_RC_OUTPUT_NUM_CHANNELS)
	return;
    uint8_t pin = _channel_to_flymaple_pin(ch);
    pwmWrite(pin, (period_us * _clocks_per_msecond[ch]) / 1000);
}

void FLYMAPLERCOutput::write(uint8_t ch, uint16_t* period_us, uint8_t len)
{
    for (int i = 0; i < len; i++)
        write(i + ch, period_us[i]); 
}

uint16_t FLYMAPLERCOutput::read(uint8_t ch) 
{
    if (ch >= FLYMAPLE_RC_OUTPUT_NUM_CHANNELS)
	return 0;
    uint8_t pin = _channel_to_flymaple_pin(ch);
    timer_dev *tdev = PIN_MAP[pin].timer_device;
    uint8 timer_channel = PIN_MAP[pin].timer_channel;
    __io uint32 *ccr = &(tdev->regs).gen->CCR1 + (timer_channel - 1);
    return *ccr * 1000 / _clocks_per_msecond[ch];
}

void FLYMAPLERCOutput::read(uint16_t* period_us, uint8_t len)
{
    for (int i = 0; i < len; i++)
        period_us[i] = read(i);
}

uint8_t FLYMAPLERCOutput::_channel_to_flymaple_pin(uint8_t ch)
{
    // This maps the ArduPilot channel numbers to Flymaple PWM output pins
    // Channels on the same timer ALWAYS use the same frequency (the last one set)
    // 28, 27, 11, 12 use Timer 3 OK
    // 24, 14, 5,  9  use Timer 4 BREAKS I2C on pins 5 and 9
    // 35, 36, 37, 38 use Timer 8 DONT USE: CRASHES. WHY?
    // 0   1,  2,  3  use Timer 2 OK
    static uint8_t ch_to_pin[FLYMAPLE_RC_OUTPUT_NUM_CHANNELS] = { 28, 27, 11, 12, 24, 14 };
    if (ch >= FLYMAPLE_RC_OUTPUT_NUM_CHANNELS)
	return 0; // Should never happen. REVISIT?
    else
	return ch_to_pin[ch];
}

void FLYMAPLERCOutput::_set_freq(uint8_t ch, uint16_t freq_hz) 
{
    if (ch >= FLYMAPLE_RC_OUTPUT_NUM_CHANNELS)
	return;
    if (freq_hz == 0)
	return; // Silly, avoid divide by 0 later
    uint8_t pin = _channel_to_flymaple_pin(ch);
    timer_dev *tdev = PIN_MAP[pin].timer_device;

    if (tdev == NULL)
        return; // Should never happen

    uint32 microseconds = 1000000 / freq_hz; // per period
    uint32 period_cyc = microseconds * CYCLES_PER_MICROSECOND; // system clock cycles per period
    // This picks the smallest prescaler that allows an overflow < 2^16.
    uint16 prescaler = (uint16)(period_cyc / MAX_OVERFLOW + 1);
    uint16 overflow = (uint16)(period_cyc / (prescaler+1));
    _clocks_per_msecond[ch] = F_CPU / (prescaler+1) / 1000;
    timer_pause(tdev);
    timer_set_prescaler(tdev, prescaler);
    timer_set_reload(tdev, overflow);
    timer_generate_update(tdev);
    timer_resume(tdev);
}
#endif
