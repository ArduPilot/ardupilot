
#ifndef __FOLLOWME_ARDUCOPTER_DEFINES_H__
#define __FOLLOWME_ARDUCOPTER_DEFINES_H__

/* These have been taken from the ArduCopter/defines.h. Kinda wish they were
 * globally accessible...
 * prefixed with MODE_ for namespacing. */

// Auto Pilot modes
// ----------------
#define MODE_STABILIZE 0                     // hold level position
#define MODE_ACRO 1                          // rate control
#define MODE_ALT_HOLD 2                      // AUTO control
#define MODE_AUTO 3                          // AUTO control
#define MODE_GUIDED 4                        // AUTO control
#define MODE_LOITER 5                        // Hold a single location
#define MODE_RTL 6                           // AUTO control
#define MODE_CIRCLE 7                        // AUTO control
#define MODE_POSITION 8                      // AUTO control
#define MODE_LAND 9                          // AUTO control
#define MODE_OF_LOITER 10            // Hold a single location using optical flow
                                // sensor
#define MODE_TOY_A 11                                // THOR Enum for Toy mode
#define MODE_TOY_M 12                                // THOR Enum for Toy mode
#define MODE_NUM_MODES 13

#endif // 

