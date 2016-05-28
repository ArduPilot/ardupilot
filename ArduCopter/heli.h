/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __HELI_H__
#define __HELI_H__

// Traditional helicopter variables and functions

#if FRAME_CONFIG == HELI_FRAME

// Mode filter to reject RC Input glitches.  Filter size is 5, and it draws the 4th element, so it can reject 3 low glitches,
// and 1 high glitch.  This is because any "off" glitches can be highly problematic for a helicopter running an ESC
// governor.  Even a single "off" frame can cause the rotor to slow dramatically and take a long time to restart.
ModeFilterInt16_Size5 rotor_speed_deglitch_filter(4);

// Tradheli flags
static struct {
    uint8_t dynamic_flight          : 1;    // 0   // true if we are moving at a significant speed (used to turn on/off leaky I terms)
    uint8_t init_targets_on_arming  : 1;    // 1   // true if we have been disarmed, and need to reset rate controller targets when we arm
} heli_flags;

#endif  // FRAME_CONFIG == HELI_FRAME
#endif  // __HELI_H__