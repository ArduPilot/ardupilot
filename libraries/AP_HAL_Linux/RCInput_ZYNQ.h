
#ifndef __AP_HAL_LINUX_RCINPUT_ZYNQ_H__
#define __AP_HAL_LINUX_RCINPUT_ZYNQ_H__

/*
  This class implements RCInput on the ZYNQ / ZyboPilot platform with custom
  logic doing the edge detection of the PPM sum input
 */

#include <AP_HAL_Linux.h>

// FIXME A puppie dies when you hard code an address
#define RCIN_ZYNQ_PULSE_INPUT_BASE  0x43c10000

class Linux::LinuxRCInput_ZYNQ : public Linux::LinuxRCInput 
{
public:
    void init(void*);
    void _timer_tick(void);

 private:
    static const int TICK_PER_US=100;
    static const int TICK_PER_S=100000000;

    // Memory mapped keyhole register to pulse input FIFO
    volatile uint32_t *pulse_input;

    // time spent in the low state
    uint32_t _s0_time;
};

#endif // __AP_HAL_LINUX_RCINPUT_ZYNQ_H__
