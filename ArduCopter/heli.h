/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __HELI_H__
#define __HELI_H__

// Traditional helicopter variables and functions

#if FRAME_CONFIG == HELI_FRAME

// Tradheli flags
static struct {
    uint8_t dynamic_flight          : 1;    // 0   // true if we are moving at a significant speed (used to turn on/off leaky I terms)
    uint8_t init_targets_on_arming  : 1;    // 1   // true if we have been disarmed, and need to reset rate controller targets when we arm
} heli_flags;

#endif  // FRAME_CONFIG == HELI_FRAME
#endif  // __HELI_H__