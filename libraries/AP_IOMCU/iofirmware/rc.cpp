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
  SBUS output support
 */

#include "ch.h"
#include "hal.h"
#include "iofirmware.h"
#include "rc.h"
#include <AP_SBusOut/AP_SBusOut.h>

extern const AP_HAL::HAL& hal;

static const SerialConfig uart3_cfg = {
    100000,   // speed
    USART_CR1_PCE | USART_CR1_M, // cr1, enable even parity
    0,        // cr2
    0,        // cr3
    nullptr,  // irq_cb
    nullptr,  // ctx
};

void sbus_out_init(void)
{
    sdStart(&SD3, &uart3_cfg);
}

void sbus_out_write(uint16_t *channels, uint8_t nchannels)
{
    uint8_t buffer[25];
    AP_SBusOut::sbus_format_frame(channels, nchannels, buffer);
    chnWrite(&SD3, buffer, sizeof(buffer));
}

/*
  sleep for 1ms using a busy loop
 */
static void delay_one_ms(uint32_t &now)
{
    while (now == AP_HAL::millis()) ;
    now = AP_HAL::millis();
}

/*
  perform a DSM bind operation
 */
void AP_IOMCU_FW::dsm_bind_step(void)
{
    uint32_t now = last_ms;
    switch (dsm_bind_state) {
    case 1:
        palSetLineMode(HAL_GPIO_PIN_SPEKTRUM_PWR_EN, PAL_MODE_OUTPUT_PUSHPULL);
        palClearLine(HAL_GPIO_PIN_SPEKTRUM_PWR_EN);
        palSetLineMode(HAL_GPIO_PIN_SPEKTRUM_OUT, PAL_MODE_OUTPUT_PUSHPULL);
        palSetLine(HAL_GPIO_PIN_SPEKTRUM_OUT);
        dsm_bind_state = 2;
        last_dsm_bind_ms = now;
        break;

    case 2:
        if (now - last_dsm_bind_ms >= 500) {
            palSetLine(HAL_GPIO_PIN_SPEKTRUM_PWR_EN);
            dsm_bind_state = 3;
            last_dsm_bind_ms = now;
        }
        break;

    case 3: {
        if (now - last_dsm_bind_ms >= 72) {
            // 9 pulses works with all satellite receivers, and supports the highest
            // available protocol
            delay_one_ms(now);
            const uint8_t num_pulses = 9;
            for (uint8_t i=0; i<num_pulses; i++) {
                // the delay should be 120us, but we are running our
                // clock at 1kHz, and 1ms works fine
                delay_one_ms(now);
                palClearLine(HAL_GPIO_PIN_SPEKTRUM_OUT);
                delay_one_ms(now);
                palSetLine(HAL_GPIO_PIN_SPEKTRUM_OUT);
            }
            last_dsm_bind_ms = now;
            dsm_bind_state = 4;
        }
        break;
    }

    case 4:
        if (now - last_dsm_bind_ms >= 50) {
            // set back as alternate function with pullup for UART input
            palSetLineMode(HAL_GPIO_PIN_SPEKTRUM_OUT, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
            palSetLine(HAL_GPIO_PIN_SPEKTRUM_OUT);
            dsm_bind_state = 0;
        }
        break;

    default:
        dsm_bind_state = 0;
        break;
    }
}

