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
#pragma once
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_HAL/AP_HAL_Boards.h>

#include "AP_HAL_ChibiOS.h"

#if HAL_USE_ICU == TRUE

#define INPUT_CAPTURE_FREQUENCY 1000000 //capture unit in microseconds

#ifndef SOFTSIG_MAX_SIGNAL_TRANSITIONS
#define SOFTSIG_MAX_SIGNAL_TRANSITIONS 128
#endif

// we use a small bounce buffer size to minimise time in the DMA
// callback IRQ
#define SOFTSIG_BOUNCE_BUF_SIZE 8

class ChibiOS::SoftSigReader {
    friend class ChibiOS::RCInput;
public:
    bool attach_capture_timer(ICUDriver* icu_drv, icuchannel_t chan, uint8_t dma_stream, uint32_t dma_channel);
    void disable(void);

private:
    uint32_t *signal;
    uint32_t signal2[SOFTSIG_BOUNCE_BUF_SIZE];
    static void _irq_handler(void* self, uint32_t flags);
    uint8_t num_timer_channels;
    uint8_t enable_chan_mask;
    uint8_t max_pulse_width;
    const stm32_dma_stream_t* dma;
    uint32_t dmamode;
    ICUConfig icucfg;
    ICUDriver* _icu_drv = nullptr;
    typedef struct PACKED {
        uint32_t w0;
        uint32_t w1;
    } pulse_t;
    ObjectBuffer<pulse_t> sigbuf{SOFTSIG_MAX_SIGNAL_TRANSITIONS};
    bool need_swap;
};

#endif // HAL_USE_ICU
