/*
 * prucomm.h - structure definitions for communication
 *
 */
#ifndef PRUCOMM_H
#define PRUCOMM_H

#include "pru_defs.h"

#define NUM_RING_ENTRIES 300
    
#define PWM_CMD_MAGIC    0xf00fbaaf
#define PWM_REPLY_MAGIC  0xbaaff00f

struct ring_buffer {
    volatile uint16_t ring_head;
    volatile uint16_t ring_tail;
    struct {
           uint16_t pin_value;
           uint16_t delta_t;
    } buffer[NUM_RING_ENTRIES];
};

/* the command is at the start of shared DPRAM */
#define RBUFF        ((volatile struct ring_buffer *)DPRAM_SHARED)

#endif
