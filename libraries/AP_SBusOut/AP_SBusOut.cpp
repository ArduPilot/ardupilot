/*
 * AP_SBusOut.cpp
 *
 *  Created on: Aug 19, 2017
 *      Author: Mark Whitehorn
 *
 * method sbus1_out was ported from ardupilot/modules/PX4Firmware/src/lib/rc/sbus.c
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

#include "AP_SBusOut_config.h"

#if AP_SBUSOUTPUT_ENABLED

#include "AP_SBusOut.h"
#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL& hal;

#define SBUS_DEBUG 0

#define SBUS_BSIZE 25
#define SBUS_CHANNELS 16
#define SBUS_MIN 875

const AP_Param::GroupInfo AP_SBusOut::var_info[] = {
    // @Param: RATE
    // @DisplayName: SBUS default output rate
    // @Description: This sets the SBUS output frame rate in Hz.
    // @Range: 25 250
    // @User: Advanced
    // @Units: Hz
    AP_GROUPINFO("RATE",  1, AP_SBusOut, sbus_rate, 50),

    AP_GROUPEND
};


// constructor
AP_SBusOut::AP_SBusOut(void)
{
    // set defaults from the parameter table
    AP_Param::setup_object_defaults(this, var_info);
}

/*
  format a SBUS output frame into a 25 byte buffer
 */
void AP_SBusOut::sbus_format_frame(uint16_t *channels, uint8_t num_channels, uint8_t buffer[SBUS_BSIZE])
{
    uint8_t index = 1;
    uint8_t offset = 0;

    memset(buffer, 0, SBUS_BSIZE);
    buffer[0] = 0x0f;

    /* construct sbus frame representing channels 1 through 16 (max) */
    const uint8_t nchan = MIN(num_channels, SBUS_CHANNELS);
    for (unsigned i = 0; i < nchan; ++i) {
        /*protect from out of bounds values and limit to 11 bits*/
        const uint16_t pwmval = MAX(channels[i], SBUS_MIN);
        const uint32_t v1 = uint32_t(pwmval - SBUS_MIN) * 1600U;
        uint16_t value = uint16_t(v1 / 1000U) + 1;
        if (value > 0x07ff) {
            value = 0x07ff;
        }

#if SBUS_DEBUG
        static uint16_t lastch9 = 0;
        if ((i==8) && (pwmval != lastch9)) {
            lastch9 = pwmval;
            hal.console->printf("channel 9 pwm: %04d\n", pwmval);
        }
#endif

        while (offset >= 8) {
            ++index;
            offset -= 8;
        }

        buffer[index] |= (value << (offset)) & 0xff;
        buffer[index + 1] |= (value >> (8 - offset)) & 0xff;
        buffer[index + 2] |= (value >> (16 - offset)) & 0xff;
        offset += 11;
    }
}

/*
 * build and send sbus1 frame representing first 16 servo channels
 * input arg is pointer to uart
 */
void
AP_SBusOut::update()
{
    if (!initialised) {
        initialised = true;
        init();
    }

    if (sbus1_uart == nullptr) {
        return;
    }

    // constrain output rate using sbus_frame_interval
    static uint32_t last_micros = 0;
    uint32_t now = AP_HAL::micros();
    if ((now - last_micros) <= sbus_frame_interval) {
        return;
    }

    last_micros = now;

    /* construct sbus frame representing channels 1 through 16 (max) */
    uint8_t nchan = MIN(NUM_SERVO_CHANNELS, SBUS_CHANNELS);
    uint16_t channels[SBUS_CHANNELS] {};

    for (unsigned i = 0; i < nchan; ++i) {
        SRV_Channel *c = SRV_Channels::srv_channel(i);
        if (c == nullptr) {
            continue;
        }
        channels[i] = c->get_output_pwm();
    }
    uint8_t buffer[SBUS_BSIZE];
    sbus_format_frame(channels, nchan, buffer);

#if SBUS_DEBUG
    hal.gpio->pinMode(55, HAL_GPIO_OUTPUT);
    hal.gpio->write(55, 1);
#endif

    sbus1_uart->write(buffer, sizeof(buffer));

#if SBUS_DEBUG
    hal.gpio->pinMode(55, HAL_GPIO_OUTPUT);
    hal.gpio->write(55, 0);
#endif
}

void AP_SBusOut::init() {
    uint16_t rate = sbus_rate.get();

#if SBUS_DEBUG
    hal.console->printf("AP_SBusOut: init %d Hz\n", rate);
#endif

    // subtract 500usec from requested frame interval to allow for latency
    sbus_frame_interval = (1000UL * 1000UL) / rate - 500;
    // at 100,000 bps, a 300 bit sbus frame takes 3msec to transfer
    // require a minimum 700usec interframe gap
    if (sbus_frame_interval < 3700) {
        sbus_frame_interval = 3700;
    }

    AP_SerialManager *serial_manager = AP_SerialManager::get_singleton();
    if (!serial_manager) {
        return;
    }
    sbus1_uart = serial_manager->find_serial(AP_SerialManager::SerialProtocol_Sbus1,0);

    if (sbus1_uart == nullptr) {
        return;
    }

    // update baud param in case user looks at it
    serial_manager->set_and_default_baud(AP_SerialManager::SerialProtocol_Sbus1, 0, 100000);

    auto &uart = *sbus1_uart;

    uart.begin(100000, 16, 32);
    uart.configure_parity(2);    // enable even parity
    uart.set_stop_bits(2);
    uart.set_unbuffered_writes(true);
    uart.set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
}

#endif  // AP_SBUSOUTPUT_ENABLED
