#pragma once

// board-specific values for this subsystem are generated into a hwdef
// fragment so they only enter the include closure of code using them
#if __has_include(<hwdef_parachute.h>)
#include <hwdef_parachute.h>
#endif


#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_PARACHUTE_ENABLED
// default to parachute enabled to match previous configs
#define HAL_PARACHUTE_ENABLED 1
#endif
