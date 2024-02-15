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
#pragma once

#include <AP_HAL/AP_HAL.h>

#ifndef AP_LTM_TELEM_ENABLED
#define AP_LTM_TELEM_ENABLED 1
#endif

#if AP_LTM_TELEM_ENABLED

static const uint8_t LTM_GFRAME_SIZE = 18;
static const uint8_t LTM_AFRAME_SIZE = 10;
static const uint8_t LTM_SFRAME_SIZE = 11;

class AP_LTM_Telem {
public:
    // Constructor
    AP_LTM_Telem() {}

    /* Do not allow copies */
    CLASS_NO_COPY(AP_LTM_Telem);

    // init - perform required initialisation
    void init();

private:
    AP_HAL::UARTDriver *_port;              // UART

    uint8_t _ltm_scheduler;
    uint32_t _last_frame_ms;

    void send_Gframe(void);
    void send_Sframe(void);
    void send_Aframe(void);
    void generate_LTM(void);
    void send_LTM(uint8_t lt_packet[], uint8_t lt_packet_size);
    void tick(void); // tick - main call to send updates to transmitter (called by scheduler at 1kHz)
};

#endif
