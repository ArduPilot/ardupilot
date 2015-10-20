#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
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

void RCInput_ZYNQ::init(void*)
{
    int mem_fd = open("/dev/mem", O_RDWR|O_SYNC);
    if (mem_fd == -1) {
        hal.scheduler->panic("Unable to open /dev/mem");
    }
    pulse_input = (volatile uint32_t*) mmap(0, 0x1000, PROT_READ|PROT_WRITE, 
                                                      MAP_SHARED, mem_fd, RCIN_ZYNQ_PULSE_INPUT_BASE);
    close(mem_fd);

    _s0_time = 0;
}

/*
  called at 1kHz to check for new pulse capture data from the PL pulse timer
 */
void RCInput_ZYNQ::_timer_tick()
{
    uint32_t v;

    // all F's means no samples available
    while((v = *pulse_input) != 0xffffffff) {
        // Hi bit indicates pin state, low bits denote pulse length
        if(!(v & 0x80000000))
            _s0_time = (v & 0x7fffffff)/TICK_PER_US;
        else
            _process_rc_pulse(_s0_time, (v & 0x7fffffff)/TICK_PER_US);
    }
}

#endif // CONFIG_HAL_BOARD
