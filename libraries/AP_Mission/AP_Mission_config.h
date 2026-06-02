#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_MISSION_ENABLED
#define AP_MISSION_ENABLED 1
#endif

#ifndef AP_MISSION_NAV_PAYLOAD_PLACE_ENABLED
#define AP_MISSION_NAV_PAYLOAD_PLACE_ENABLED 1
#endif

// No handling of MISSION_ITEM is present in the AP_Mission library -
// everything is MISSION_ITEM_INT.  However, there is a pair of static
// methods which allows conversion from a mavlink MISSION_ITEM message
// to a MISSION_ITEM_INT method and vice-versa, and they are gated on
// this define.  Additionally, the GCS library uses this define to
// gate acceptance of various mavlink messages (the GCS library also
// being the user of the static methods)
// CODE_REMOVAL
// ArduPilot 4.4 sends a warning if MISSION_ITEM is handled
// ArduPilot 4.8 compiles the code out by default
// ArduPilot 4.9 removes the code entirely
#ifndef AP_MISSION_ITEM_ENABLED
#define AP_MISSION_ITEM_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif  // AP_MISSION_ITEM_ENABLED
