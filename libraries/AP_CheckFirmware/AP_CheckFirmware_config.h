#pragma once

// board-specific values for this subsystem are generated into a hwdef
// fragment so they only enter the include closure of code using them
#if __has_include(<hwdef_caps.h>)
#include <hwdef_caps.h>
#endif

#ifndef AP_BOOTLOADER_FLASHING_ENABLED
#define AP_BOOTLOADER_FLASHING_ENABLED 0
#endif
