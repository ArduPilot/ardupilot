#pragma once

/*
  This class implements RCInput on the BeagleBoneBlack with a PRU
  doing the edge detection of the PPM sum input
 */

#include "AP_HAL_Linux.h"

#define RCIN_PRUSS_SHAREDRAM_BASE   0x4a312000
// we use 300 ring buffer entries to guarantee that a full 25 byte
// frame of 12 bits per byte

class Linux::RCInput_PRU : public Linux::RCInput
{
public:
    void init();
    void _timer_tick(void);

 private:
    static const unsigned int NUM_RING_ENTRIES=300;
    // shared ring buffer with the PRU which records pin transitions
    struct ring_buffer {
        volatile uint16_t ring_head; // owned by ARM CPU
        volatile uint16_t ring_tail; // owned by the PRU
        struct {
               uint16_t pin_value;
               uint16_t delta_t;
        } buffer[NUM_RING_ENTRIES];
    };
    volatile struct ring_buffer *ring_buffer;

    // time spent in the low state
    uint16_t _s0_time;
};
