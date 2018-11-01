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

static bool sbus_out_initialised;

// usart3 is for SBUS input and output
static const SerialConfig uart3_cfg = {
    100000,   // speed
    USART_CR1_PCE | USART_CR1_M, // cr1, enable even parity
    0,        // cr2
    0,        // cr3
    nullptr,  // irq_cb
    nullptr,  // ctx
};

void sbus_out_write(uint16_t *channels, uint8_t nchannels)
{
    if (!sbus_out_initialised) {
        sdStart(&SD3, &uart3_cfg);
        sbus_out_initialised = true;
    }
    uint8_t buffer[25];
    AP_SBusOut::sbus_format_frame(channels, nchannels, buffer);
    chnWrite(&SD3, buffer, sizeof(buffer));
}

// usart1 is for DSM input and (optionally) debug to FMU
static const SerialConfig uart1_cfg = {
    115200,   // speed
    0,        // cr1
    0,        // cr2
    0,        // cr3
    nullptr,  // irq_cb
    nullptr,  // ctx
};

/*
  init rcin on DSM USART1
 */
void AP_IOMCU_FW::rcin_serial_init(void)
{
    sdStart(&SD1, &uart1_cfg);
}


static uint32_t num_dsm_bytes;

/*
  check for data on DSM RX uart
 */
void AP_IOMCU_FW::rcin_serial_update(void)
{
    uint8_t b[16];
    uint32_t n;
    if ((n = chnReadTimeout(&SD1, b, sizeof(b), TIME_IMMEDIATE)) > 0) {
        n = MIN(n, sizeof(b));
        num_dsm_bytes += n;
        for (uint8_t i=0; i<n; i++) {
            rcprotocol->process_byte(b[i]);
        }
        //palToggleLine(HAL_GPIO_PIN_HEATER);
    }
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
            // set back as input pin
            palSetLineMode(HAL_GPIO_PIN_SPEKTRUM_OUT, PAL_MODE_INPUT);
            dsm_bind_state = 0;
        }
        break;

    default:
        dsm_bind_state = 0;
        break;
    }
}

