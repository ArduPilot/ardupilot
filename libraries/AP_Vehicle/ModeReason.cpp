#include "ModeReason.h"

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_MODE_REASON_STRINGS
#define AP_MODE_REASON_STRINGS (BOARD_FLASH_SIZE > 1024)
#endif

#if AP_MODE_REASON_STRINGS

const char *ap_mode_reason_string(ModeReason reason)
{
    static const char *ap_mode_reason_strings[] {
        "Unknown",
        "RC Command",
        "GCS Command",
        "Radio Failsafe",
        "Battery Failsafe",
        "GCS Failsafe",
        "EKF Failsafe",
        "GPS Glitch",
        "Mission End",
        "Throttle Land Escape",
        "Fence Breached",
        "Terrain Failsafe",
        "Brake Timeout",
        "Flip Complete",
        "Avoidance",
        "Avoidance Recovery",
        "Throw Complete",
        "Terminate",
        "Toy Mode",
        "Crash Failsafe",
        "Soaring FBW B With Motor Running",
        "Soaring Thermal Detected",
        "Soaring Thermal Estimate Deteriorated",
        "VTOL Failed Transition",
        "VTOL Failed Takeoff",
        "Failsafe", // prefer specific failsafes over this as much as possible
        "Initialised",
        "Surface Complete",
        "Bad Depth",
        "Leak Failsafe",
        "Servotest",
        "Startup",
        "Scripting",
        "Unavailable",
        "Autorotation Start",
        "Autorotation Bailout",
        "Soaring Alt Too High",
        "Soaring Alt Too Low",
        "Soaring Drift Exceeded",
        "RTL Complete Switching To VTOL Land RTL",
        "RTL Complete Switching To Fixedwing Autoland",
        "Mission Cmd",
        "Frsky Command",
        "Fence Return Previous Mode",
        "QRTL Instead Of RTL",
        "Auto RTL Exit",
        "Loiter Alt Reached QLAND",
        "Loiter Alt In VTOL",
        "Radio Failsafe Recovery",
    };
    if ((uint8_t)reason >= ARRAY_SIZE(ap_mode_reason_strings)) {
        return "?";
    }
    return ap_mode_reason_strings[uint8_t(reason)];
}

#else

const char *ap_mode_reason_string(ModeReason reason) {
    return nullptr;
}

#endif
