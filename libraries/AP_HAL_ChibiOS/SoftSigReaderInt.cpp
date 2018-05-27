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
#include "hwdef/common/stm32_util.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

using namespace ChibiOS;
extern const AP_HAL::HAL& hal;

#if HAL_USE_EICU == TRUE

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
#ifdef HAL_RCIN_IS_INVERTED
    channel_config.alvl = EICU_INPUT_ACTIVE_HIGH;
#else
    channel_config.alvl = EICU_INPUT_ACTIVE_LOW;
#endif
    channel_config.capture_cb = _irq_handler;
    eicuStart(_icu_drv, &icucfg);
    //sets input filtering to 4 timer clock
    stm32_timer_set_input_filter(_icu_drv->tim, chan, 2);
    eicuEnable(_icu_drv);
}

inline void invert_polarity(EICUDriver *eicup, eicuchannel_t channel)
{
    eicup->tim->CCER ^= STM32_TIM_CCER_CC1P << (channel * 4);
}

//check for CCxOF, if it was set by timer hw, it means we missed 2 or more transitions
inline bool overcapture_occured(EICUDriver *eicup, eicuchannel_t channel)
{
    bool result = (eicup->tim->SR & (STM32_TIM_SR_CC1OF << channel)) != 0;
    eicup->tim->SR &= ~(STM32_TIM_SR_CC1OF << channel);
    return result;
}

//check for right pin polarity, if it is wrong it means we missed 1, 3, 5 or more transitions
inline bool wrong_polarity(EICUDriver *eicup, eicuchannel_t channel)
{
    bool pin_high = palReadLine(RCININT_PIN);
    bool waiting_rising = (eicup->tim->CCER & (STM32_TIM_CCER_CC1P << (channel * 4))) == 0;
    if (pin_high && waiting_rising) {
        return true;
    }
    if (!pin_high && !waiting_rising) {
        return true;
    }
    return false;
}

void SoftSigReaderInt::_irq_handler(EICUDriver *eicup, eicuchannel_t channel)
{
    uint16_t value = eicup->tim->CCR[channel];
    invert_polarity(eicup, channel);
    _instance->sigbuf.push(value);
    
    //check for missed interrupt 
    if (overcapture_occured(eicup, channel) || wrong_polarity(eicup, channel)) {
        //we have missed some pulses
        //try to reset RCProtocol parser by returning invalid value (i.e. 0 width pulse)        
        _instance->sigbuf.push(value);
        //second 0 width pulse to keep polarity right
        _instance->sigbuf.push(value);
    }
}

bool SoftSigReaderInt::read(uint32_t &widths0, uint32_t &widths1)
{
    uint16_t p0, p1;
    if (sigbuf.available() >= 2) {
        if (sigbuf.pop(p0)&&sigbuf.pop(p1)) {
            widths0 = uint16_t(p0 - last_value);
            widths1 = uint16_t(p1 - p0);
            last_value = p1;
            return true;
        }
    }
    return false;
}

#endif // HAL_USE_EICU

#endif //CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
