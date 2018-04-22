/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "SoftSigReaderInt.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

using namespace ChibiOS;
extern const AP_HAL::HAL& hal;

#if HAL_USE_EICU == TRUE

#define eicu_lld_invert_polarity(eicup, channel)                              \
  (eicup)->tim->CCER ^= ((uint16_t)(STM32_TIM_CCER_CC1P << ((channel) * 4)))

// singleton instance
SoftSigReaderInt *SoftSigReaderInt::_instance;

SoftSigReaderInt::SoftSigReaderInt()
{
    _instance = this;
}

void SoftSigReaderInt::init(EICUDriver* icu_drv, eicuchannel_t chan)
{
    last_value = 0;
    _icu_drv = icu_drv;
    icucfg.dier = 0;
    icucfg.frequency = INPUT_CAPTURE_FREQUENCY;
    for (int i=0; i< EICU_CHANNEL_ENUM_END; i++) {
        icucfg.iccfgp[i]=nullptr;
    }
    icucfg.iccfgp[chan] = &channel_config;
    channel_config.alvl = EICU_INPUT_ACTIVE_HIGH;
    channel_config.capture_cb = _irq_handler;
    eicuStart(_icu_drv, &icucfg);
    eicuEnable(_icu_drv);
}

void SoftSigReaderInt::_irq_handler(EICUDriver *eicup, eicuchannel_t channel)
{
    uint16_t value = eicup->tim->CCR[channel];
    _instance->sigbuf.push(value);
    eicu_lld_invert_polarity(eicup, channel);
}

bool SoftSigReaderInt::read(uint32_t &widths0, uint32_t &widths1)
{
    uint16_t p0, p1;
    if (sigbuf.available() >= 2) {
        if (sigbuf.pop(p0)&&sigbuf.pop(p1)) {
            widths0 = p0 - last_value;
            widths1 = p1 - p0;
            last_value = p1;
            return true;
        }
    }
    return false;
}

#endif // HAL_USE_EICU

#endif //CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
