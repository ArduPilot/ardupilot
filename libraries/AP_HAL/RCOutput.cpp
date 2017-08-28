/*
 * RCOutput.cpp
 *
 *  Created on: Aug 19, 2017
 *      Author: markw
 */

/*
   Please contribute your ideas! See http://dev.ardupilot.org for details

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

/* method sb1_out was ported from ardupilot/modules/PX4Firmware/src/lib/rc/sbus.c
 * which has the following license:
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "RCOutput.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <SRV_Channel/SRV_Channel.h>

// SBUS1 constant definitions
// pulse widths measured using FrSky Sbus/PWM converter
#define SBUS_BSIZE    25
#define SBUS_CHANNELS 16
#define SBUS_MIN 880.0f
#define SBUS_MAX 2156.0f
#define SBUS_SCALE (2048.0f / (SBUS_MAX - SBUS_MIN))

/*
 * build and send sbus1 frame representing first 16 servo channels
 * input arg is pointer to uart
 */
void
AP_HAL::RCOutput::sbus1_out(uint16_t *pwidth, uint8_t nchan)
{
    if (sbus1_uart == nullptr) return;

    // constrain output rate using sbus_frame_interval
    static uint32_t last_micros = 0;
    uint32_t now = AP_HAL::micros();
    if ((now - last_micros) > sbus_frame_interval) {
        last_micros = now;
        uint8_t buffer[SBUS_BSIZE] = { 0x0f };  // first byte is always 0x0f
        uint8_t index = 1;
        uint8_t offset = 0;
        uint16_t value;

        /* construct sbus frame representing channels 1 through 16 (max) */
        if (nchan > SBUS_CHANNELS) nchan = SBUS_CHANNELS;
        for (unsigned i = 0; i < nchan; ++i) {
            value = (uint16_t)((pwidth[i] - SBUS_MIN) * SBUS_SCALE);

            /*protect from out of bounds values and limit to 11 bits*/
            if (value > 0x07ff) {
                value = 0x07ff;
            }

            while (offset >= 8) {
                ++index;
                offset -= 8;
            }

            buffer[index] |= (value << (offset)) & 0xff;
            buffer[index + 1] |= (value >> (8 - offset)) & 0xff;
            buffer[index + 2] |= (value >> (16 - offset)) & 0xff;
            offset += 11;
        }
        sbus1_uart->write(buffer, sizeof(buffer));
    }
}

void AP_HAL::RCOutput::enable_sbus_out(UARTDriver* uart, uint16_t rate) {
    sbus1_uart = uart;
    sbus1_uart->begin(100000);
    sbus1_uart->configure_parity(2);    // enable even parity
    sbus1_uart->set_stop_bits(2);
    sbus1_uart->set_unbuffered_writes(true);
    sbus_frame_interval = (1000UL * 1000UL) / rate;
}

