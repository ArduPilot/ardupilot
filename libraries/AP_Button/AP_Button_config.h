#pragma once

// board-specific values for this subsystem are generated into a hwdef
// fragment so they only enter the include closure of code using them
#if __has_include(<hwdef_button.h>)
#include <hwdef_button.h>
#endif

#ifndef HAL_BUTTON_ENABLED
#define HAL_BUTTON_ENABLED 1
#endif
