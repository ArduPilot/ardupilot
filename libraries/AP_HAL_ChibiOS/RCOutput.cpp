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
 * Code by Andrew Tridgell and Siddharth Bharat Purohit
 */
#include "RCOutput.h"
#include <AP_Math/AP_Math.h>

using namespace ChibiOS;

extern const AP_HAL::HAL& hal;

#if HAL_WITH_IO_MCU
#include <AP_IOMCU/AP_IOMCU.h>
extern AP_IOMCU iomcu;
#endif

#define PWM_CLK_FREQ        8000000
#define PWM_US_WIDTH_FROM_CLK(x) ((PWM_CLK_FREQ/1000000)*x)
const struct ChibiRCOutput::pwm_group ChibiRCOutput::pwm_group_list[] = 
{
    //Group 1 Config
    {   //Channels in the Group and respective mapping
        {PWM_CHAN_MAP(0) , PWM_CHAN_MAP(1) , PWM_CHAN_MAP(2) , PWM_CHAN_MAP(3)}, 
        //Group Initial Config
        {
          8000000,                                  /* 8MHz PWM clock frequency.   */
          160000,                                    /* Initial PWM period 20ms.     */
          NULL,
          {
           //Channel Config
           {PWM_OUTPUT_ACTIVE_HIGH, NULL},
           {PWM_OUTPUT_ACTIVE_HIGH, NULL},
           {PWM_OUTPUT_ACTIVE_HIGH, NULL},
           {PWM_OUTPUT_ACTIVE_HIGH, NULL}
          },
          0,
          0
        },
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_CHIBIOS_FMUV3
        &PWMD1
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_CHIBIOS_SKYVIPER_F412
        &PWMD3
#endif
    }
};

void ChibiRCOutput::init()
{
    _num_groups = sizeof(pwm_group_list)/sizeof(pwm_group);
    for (uint8_t i = 0; i < _num_groups; i++ ) {
        //Start Pwm groups
        pwmStart(pwm_group_list[i].pwm_drv, &pwm_group_list[i].pwm_cfg);
    }
#if HAL_WITH_IO_MCU
    iomcu.init();

    // with IOMCU the local channels start at 8
    chan_offset = 8;
#endif
}

void ChibiRCOutput::set_freq(uint32_t chmask, uint16_t freq_hz)
{
    //check if the request spans accross any of the channel groups
    uint8_t update_mask = 0;
    uint32_t grp_ch_mask;
    // greater than 400 doesn't give enough room at higher periods for
    // the down pulse
    if (freq_hz > 400 && _output_mode != MODE_PWM_BRUSHED) {
        freq_hz = 400;
    }

#if HAL_WITH_IO_MCU
    iomcu.set_freq(chmask, freq_hz);
#endif

    chmask >>= chan_offset;
    if (chmask == 0) {
        return;
    }
    
    for (uint8_t i = 0; i < _num_groups; i++ ) {
        grp_ch_mask = PWM_CHAN_MAP(0) | PWM_CHAN_MAP(1) | PWM_CHAN_MAP(2) | PWM_CHAN_MAP(3);
        if ((grp_ch_mask & chmask) == grp_ch_mask) {
            update_mask |= grp_ch_mask;
            pwmChangePeriod(pwm_group_list[i].pwm_drv, 
                pwm_group_list[i].pwm_cfg.frequency/freq_hz);
        }
    }
    if (chmask != update_mask) {
        hal.console->printf("RCOutput: Failed to set PWM frequency req %x set %x\n", (unsigned)chmask, (unsigned)update_mask);
    }
}

uint16_t ChibiRCOutput::get_freq(uint8_t chan)
{
#if HAL_WITH_IO_MCU
    if (chan < chan_offset) {
        return iomcu.get_freq(chan);
    }
#endif
    chan -= chan_offset;
    
    for (uint8_t i = 0; i < _num_groups; i++ ) {
        for (uint8_t j = 0; j < 4; j++) {
            if (pwm_group_list[i].chan[j] == chan) {
                return pwm_group_list[i].pwm_drv->config->frequency / pwm_group_list[i].pwm_drv->period;
            }
        }
    }
    // assume 50Hz default
    return 50;
}

void ChibiRCOutput::enable_ch(uint8_t chan)
{
    if (chan < chan_offset) {
        return;
    }
    chan -= chan_offset;

    for (uint8_t i = 0; i < _num_groups; i++ ) {
        for (uint8_t j = 0; j < 4; j++) {
            if ((pwm_group_list[i].chan[j] == chan) && !(en_mask & 1<<chan)) {
                pwmEnableChannel(pwm_group_list[i].pwm_drv, j, PWM_US_WIDTH_FROM_CLK(900));
                en_mask |= 1<<chan;
                if(_output_mode == MODE_PWM_BRUSHED) {
                    period[chan] = 0;
                } else {
                    period[chan] = 900;
                }
            }
        }
    }
}

void ChibiRCOutput::disable_ch(uint8_t chan)
{
    if (chan < chan_offset) {
        return;
    }
    chan -= chan_offset;

    for (uint8_t i = 0; i < _num_groups; i++ ) {
        for (uint8_t j = 0; j < 4; j++) {
            if (pwm_group_list[i].chan[j] == chan) {
                pwmDisableChannel(pwm_group_list[i].pwm_drv, j);
                en_mask &= ~(1<<chan);
            }
        }
    }
}

void ChibiRCOutput::write(uint8_t chan, uint16_t period_us)
{
    last_sent[chan] = period_us;
    
#if HAL_WITH_IO_MCU
    // handle IO MCU channels
    iomcu.write_channel(chan, period_us);
#endif
    if (chan < chan_offset) {
        return;
    }
    chan -= chan_offset;
    
    period[chan] = period_us;
    num_channels = MAX(chan+1, num_channels);
    if (!corked) {
        push_local();
    }
}

/*
  push values to local channels from period[] array
 */
void ChibiRCOutput::push_local(void)
{
    if (num_channels == 0) {
        return;
    }
    uint16_t outmask = (1U<<(num_channels-1));
    for (uint8_t i = 0; i < _num_groups; i++ ) {
        for (uint8_t j = 0; j < 4; j++) {
            uint8_t chan = pwm_group_list[i].chan[j];
            if (outmask & (1UL<<chan)) {
                uint32_t period_us = period[chan];
                if(_output_mode == MODE_PWM_BRUSHED) {
                    if (period_us <= _esc_pwm_min) {
                        period_us = 0;
                    } else if (period_us >= _esc_pwm_max) {
                        period_us = PWM_FRACTION_TO_WIDTH(pwm_group_list[i].pwm_drv, 1, 1);
                    } else {
                        period_us = PWM_FRACTION_TO_WIDTH(pwm_group_list[i].pwm_drv,\
                               (_esc_pwm_max - _esc_pwm_min), (period_us - _esc_pwm_min));
                    }
                    pwmEnableChannel(pwm_group_list[i].pwm_drv, j, period_us);
                } else {
                    pwmEnableChannel(pwm_group_list[i].pwm_drv, j, PWM_US_WIDTH_FROM_CLK(period_us));
                }
            }
        }
    }
}

uint16_t ChibiRCOutput::read(uint8_t chan)
{
#if HAL_WITH_IO_MCU
    if (chan < chan_offset) {
        return iomcu.read_channel(chan);
    }
#endif
    chan -= chan_offset;
    return period[chan];
}

void ChibiRCOutput::read(uint16_t* period_us, uint8_t len)
{
#if HAL_WITH_IO_MCU
    for (uint8_t i=0; i<MIN(len, chan_offset); i++) {
        period_us[i] = iomcu.read_channel(i);
    }
#endif
    if (len <= chan_offset) {
        return;
    }
    len -= chan_offset;
    period_us += chan_offset;
    
    memcpy(period_us, period, len*sizeof(uint16_t));
}

uint16_t ChibiRCOutput::read_last_sent(uint8_t chan)
{
    return last_sent[chan];
}

void ChibiRCOutput::read_last_sent(uint16_t* period_us, uint8_t len)
{
    for (uint8_t i=0; i<len; i++) {
        period_us[i] = read_last_sent(i);
    }
}
/*
  setup output mode
 */
void ChibiRCOutput::set_output_mode(enum output_mode mode)
{
    _output_mode = mode;
    if (_output_mode == MODE_PWM_BRUSHED) {
        // force zero output initially
        for (uint8_t i=chan_offset; i<chan_offset+num_channels; i++) {
            write(i, 0);
        }
    }
}

/*
  force the safety switch on, disabling PWM output from the IO board
*/
bool ChibiRCOutput::force_safety_on(void)
{
#if HAL_WITH_IO_MCU
    return iomcu.force_safety_on();
#else
    return false;
#endif
}

/*
  force the safety switch off, enabling PWM output from the IO board
*/
void ChibiRCOutput::force_safety_off(void)
{
#if HAL_WITH_IO_MCU
    iomcu.force_safety_off();
#endif
}

/*
  start corking output
 */
void ChibiRCOutput::cork(void)
{
    corked = true;
#if HAL_WITH_IO_MCU
    iomcu.cork();
#endif
}

/*
  stop corking output
 */
void ChibiRCOutput::push(void)
{
    corked = false;
    push_local();
#if HAL_WITH_IO_MCU
    iomcu.push();
#endif
}

/*
  enable sbus output
 */
bool ChibiRCOutput::enable_px4io_sbus_out(uint16_t rate_hz)
{
#if HAL_WITH_IO_MCU
    return iomcu.enable_sbus_out(rate_hz);
#else
    return false;
#endif
}
