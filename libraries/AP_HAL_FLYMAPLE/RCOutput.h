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

#ifndef __AP_HAL_FLYMAPLE_RCOUTPUT_H__
#define __AP_HAL_FLYMAPLE_RCOUTPUT_H__

#include <AP_HAL_FLYMAPLE.h>

#define FLYMAPLE_RC_OUTPUT_NUM_CHANNELS 6

class AP_HAL_FLYMAPLE_NS::FLYMAPLERCOutput : public AP_HAL::RCOutput {
    void     init(void* machtnichts);
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
    // We map channels to Flymaple pins so AP channels 1 to 6 map to
    // the conventional Flymaple PWM output pins on J5 (pins 28, 27, 11, 12, 24, 14 respectively)
    // also
    // Channel 7 -> pin 5
    // channel 8 -> pin 9
    uint8_t _channel_to_flymaple_pin(uint8_t ch);

    void _set_freq(uint8_t ch, uint16_t freq_hz);

    uint32_t _clocks_per_msecond[FLYMAPLE_RC_OUTPUT_NUM_CHANNELS];
};

#endif // __AP_HAL_FLYMAPLE_RCOUTPUT_H__
