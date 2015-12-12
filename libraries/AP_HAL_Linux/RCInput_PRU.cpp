#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLEBOARD || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI

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

#include "GPIO.h"
#include "RCInput.h"

extern const AP_HAL::HAL& hal;

using namespace Linux;

void RCInput_PRU::init()
{
    int mem_fd = open("/dev/mem", O_RDWR|O_SYNC);
    if (mem_fd == -1) {
        AP_HAL::panic("Unable to open /dev/mem");
    }
    ring_buffer = (volatile struct ring_buffer*) mmap(0, 0x1000, PROT_READ|PROT_WRITE, 
                                                      MAP_SHARED, mem_fd, RCIN_PRUSS_SHAREDRAM_BASE);
    close(mem_fd);
    ring_buffer->ring_head = 0;
    _s0_time = 0;

    // enable the spektrum RC input power
    hal.gpio->pinMode(BBB_P8_17, HAL_GPIO_OUTPUT);
    hal.gpio->write(BBB_P8_17, 1);
}

/*
  called at 1kHz to check for new pulse capture data from the PRU
 */
void RCInput_PRU::_timer_tick()
{
    while (ring_buffer->ring_head != ring_buffer->ring_tail) {
        if (ring_buffer->ring_tail >= NUM_RING_ENTRIES) {
            // invalid ring_tail from PRU - ignore RC input
            return;
        }
        if (ring_buffer->buffer[ring_buffer->ring_head].pin_value == 1) {
            // remember the time we spent in the low state
            _s0_time = ring_buffer->buffer[ring_buffer->ring_head].delta_t;
        } else {
            // the pulse value is the sum of the time spent in the low
            // and high states
            _process_rc_pulse(_s0_time, ring_buffer->buffer[ring_buffer->ring_head].delta_t);
        }
        // move to the next ring buffer entry
        ring_buffer->ring_head = (ring_buffer->ring_head + 1) % NUM_RING_ENTRIES;        
    }
}

#endif // CONFIG_HAL_BOARD_SUBTYPE
