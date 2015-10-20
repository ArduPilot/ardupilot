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


#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI

#include <stdio.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <stdint.h>

#include "RCInput.h"
#include "RCInput_AioPRU.h"

extern const AP_HAL::HAL& hal;

using namespace Linux;

void RCInput_AioPRU::init(void*)
{
    int mem_fd = open("/dev/mem", O_RDWR|O_SYNC);
    if (mem_fd == -1) {
        hal.scheduler->panic("Unable to open /dev/mem");
    }
    ring_buffer = (volatile struct ring_buffer*) mmap(0, 0x1000, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, RCIN_PRUSS_RAM_BASE);
    close(mem_fd);
    ring_buffer->ring_head = 0;
}

/*
  called at 1kHz to check for new pulse capture data from the PRU
 */
void RCInput_AioPRU::_timer_tick()
{
    while (ring_buffer->ring_head != ring_buffer->ring_tail) {
        if (ring_buffer->ring_tail >= NUM_RING_ENTRIES) {
            // invalid ring_tail from PRU - ignore RC input
            return;
        }
        _process_rc_pulse((ring_buffer->buffer[ring_buffer->ring_head].s1_t) / TICK_PER_US,
                          (ring_buffer->buffer[ring_buffer->ring_head].s0_t) / TICK_PER_US);
        // move to the next ring buffer entry
        ring_buffer->ring_head = (ring_buffer->ring_head + 1) % NUM_RING_ENTRIES;
    }
}

#endif // CONFIG_HAL_BOARD_SUBTYPE
