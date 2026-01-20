#pragma once

/*
  This class implements RCInput on the ZYNQ / ZyboPilot platform with custom
  logic doing the edge detection of the PPM sum input
 */

#include "RCInput.h"

#if AP_RCPROTOCOL_ZYNQ_ENABLED

#ifndef RCIN_ZYNQ_TICK_PER_US
#define RCIN_ZYNQ_TICK_PER_US 100
#endif

namespace Linux {

class RCInput_ZYNQ : public RCInput {
public:
    void init() override;
    void _timer_tick(void) override;

private:
    static const int TICK_PER_US = RCIN_ZYNQ_TICK_PER_US;
    static const int TICK_PER_S = RCIN_ZYNQ_TICK_PER_US * 1e6;

    // Memory mapped keyhole register to pulse input FIFO
    volatile uint32_t *pulse_input;

    // time spent in the low state
    uint32_t _s0_time;
};

}

#endif  // AP_RCPROTOCOL_ZYNQ_ENABLED
