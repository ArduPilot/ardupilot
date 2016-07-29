// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.
#pragma once

/*
  This class implements RCInput on the BeagleBoneBlack with a PRU
  doing the edge detection of the PPM sum input
 */

#include "AP_HAL_Linux.h"

#define RCIN_PRUSS_RAM_BASE   0x4a303000

// we use 300 ring buffer entries to guarantee that a full 25 byte
// frame of 12 bits per byte

namespace Linux {

class RCInput_AioPRU : public RCInput {
public:
    void init();
    void _timer_tick(void);

protected:
    static const uint32_t TICK_PER_US = 200;
    static const uint32_t NUM_RING_ENTRIES = 300;
    // shared ring buffer with the PRU which records pin transitions
    struct ring_buffer {
        volatile uint16_t ring_head; // owned by ARM CPU
        volatile uint16_t ring_tail; // owned by the PRU
        struct {
               volatile uint32_t s1_t; // 5ns per tick
               volatile uint32_t s0_t; // 5ns per tick
        } buffer[NUM_RING_ENTRIES];
    };
    volatile struct ring_buffer *ring_buffer;
};

}
