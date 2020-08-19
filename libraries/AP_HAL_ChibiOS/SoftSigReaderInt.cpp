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

#if STM32_EICU_USE_TIM10 || STM32_EICU_USE_TIM11 || STM32_EICU_USE_TIM13 || STM32_EICU_USE_TIM14
#error "Timers with only one channel are not supported"
#endif

// singleton instance
SoftSigReaderInt *SoftSigReaderInt::_singleton;

SoftSigReaderInt::SoftSigReaderInt()
{
    _singleton = this;
}

eicuchannel_t SoftSigReaderInt::get_pair_channel(eicuchannel_t channel)
{
    switch (channel) {
        case EICU_CHANNEL_1:
            return EICU_CHANNEL_2;
        case EICU_CHANNEL_2:
            return EICU_CHANNEL_1;
        case EICU_CHANNEL_3:
            return EICU_CHANNEL_4;
        case EICU_CHANNEL_4:
            return EICU_CHANNEL_3;
        case EICU_CHANNEL_ENUM_END:
            return EICU_CHANNEL_ENUM_END;
    }
    return EICU_CHANNEL_ENUM_END;
}

void SoftSigReaderInt::init(EICUDriver* icu_drv, eicuchannel_t chan)
{
    last_value = 0;
    _icu_drv = icu_drv;
    eicuchannel_t aux_chan = get_pair_channel(chan);
    icucfg.dier = 0;
    icucfg.frequency = INPUT_CAPTURE_FREQUENCY;
    for (int i=0; i< EICU_CHANNEL_ENUM_END; i++) {
        icucfg.iccfgp[i]=nullptr;
    }

    //configure main channel
    icucfg.iccfgp[chan] = &channel_config;
#ifdef HAL_RCIN_IS_INVERTED
    channel_config.alvl = EICU_INPUT_ACTIVE_HIGH;
#else
    channel_config.alvl = EICU_INPUT_ACTIVE_LOW;
#endif
    channel_config.capture_cb = nullptr;

    //configure aux channel
    icucfg.iccfgp[aux_chan] = &aux_channel_config;
#ifdef HAL_RCIN_IS_INVERTED
    aux_channel_config.alvl = EICU_INPUT_ACTIVE_LOW;
#else
    aux_channel_config.alvl = EICU_INPUT_ACTIVE_HIGH;
#endif
    aux_channel_config.capture_cb = _irq_handler;

    eicuStart(_icu_drv, &icucfg);
    //sets input filtering to 4 timer clock
    stm32_timer_set_input_filter(_icu_drv->tim, chan, 2);
    //sets input for aux_chan
    stm32_timer_set_channel_input(_icu_drv->tim, aux_chan, 2);
    eicuEnable(_icu_drv);
}

void SoftSigReaderInt::disable(void)
{
    eicuDisable(_icu_drv);
}

void SoftSigReaderInt::_irq_handler(EICUDriver *eicup, eicuchannel_t aux_channel)
{
    eicuchannel_t channel = get_pair_channel(aux_channel);
    pulse_t pulse;
    pulse.w0 = eicup->tim->CCR[channel];
    pulse.w1 = eicup->tim->CCR[aux_channel];

    _singleton->sigbuf.push(pulse);

    //check for missed interrupt
    uint32_t mask = (STM32_TIM_SR_CC1OF << channel) | (STM32_TIM_SR_CC1OF << aux_channel);
    if ((eicup->tim->SR & mask) != 0) {
        //we have missed some pulses
        //try to reset RCProtocol parser by returning invalid value (i.e. 0 width pulse)
        pulse.w0 = 0;
        pulse.w1 = 0;
        _singleton->sigbuf.push(pulse);
        //reset overcapture mask
        eicup->tim->SR &= ~mask;
    }
}

bool SoftSigReaderInt::read(uint32_t &widths0, uint32_t &widths1)
{
    if (sigbuf.available() >= 2) {
        pulse_t pulse;
        if (sigbuf.pop(pulse)) {
            widths0 = uint16_t(pulse.w0 - last_value);
            widths1 = uint16_t(pulse.w1 - pulse.w0);
            last_value = pulse.w1;
            return true;
        }
    }
    return false;
}

#endif // HAL_USE_EICU

#endif //CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
