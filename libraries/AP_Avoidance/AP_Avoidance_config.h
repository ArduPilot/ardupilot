#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_ADSB/AP_ADSB.h>
#include <AP_Scripting/AP_Scripting_config.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AC_Avoidance/AC_Avoidance_config.h>

#ifndef AP_ADSB_AVOIDANCE_ENABLED
#define AP_ADSB_AVOIDANCE_ENABLED HAL_ADSB_ENABLED
#endif  // AP_ADSB_AVOIDANCE_ENABLED

// Scripted detect-and-avoid.  The whole feature - the AP_OAScripting library, the
// AP_Avoidance distance queries it calls, and the AVD_ standoff parameters those
// queries read - is one unit and must compile in or out together.  Splitting it
// would let a build enable the library while its inputs were compiled away, which
// is a link error rather than a smaller binary.
//
// It costs around 8.5 kB of flash, so it is NOT in a default build: the program-size
// term restricts it to targets with more than 2 MB of program space - SITL, Linux,
// and boards carrying external program flash such as CubeRedPrimary.  Everywhere
// else it is opt-in, via the Custom Build Server ("Enable Scripted Detect and Avoid
// (DAA)" under Plane) or "waf configure --enable-AP_OASCRIPTING"; both define the
// macro explicitly and so skip the #ifndef below.
//
// This lives in a config header, rather than in AP_Avoidance.h or AP_OAScripting.h,
// because waf refuses vehicle-dependent macros in library headers unless the header
// is whitelisted in Tools/ardupilotwaf/ap_library.py - this file is.  Note that
// AP_Avoidance.cpp must keep spelling APM_BUILD_TYPE out in full: waf decides
// whether to compile a source per-vehicle by searching the .cpp text for that token,
// and it does not follow macros through headers.
#ifndef AP_OA_SCRIPTING_ENABLED
#define AP_OA_SCRIPTING_ENABLED (AP_SCRIPTING_ENABLED && APM_BUILD_TYPE(APM_BUILD_ArduPlane) && AP_AVOIDANCE_ENABLED && AP_ADSB_AVOIDANCE_ENABLED && (HAL_PROGRAM_SIZE_LIMIT_KB > 2048))
#endif  // AP_OA_SCRIPTING_ENABLED
