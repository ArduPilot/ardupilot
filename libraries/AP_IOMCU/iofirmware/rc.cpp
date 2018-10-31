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
#include "rc.h"
#include <AP_SBusOut/AP_SBusOut.h>

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
