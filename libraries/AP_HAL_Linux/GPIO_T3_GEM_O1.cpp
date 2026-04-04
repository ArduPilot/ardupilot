#include <AP_HAL/AP_HAL_Boards.h>

#include "GPIO_T3_GEM_O1.h"

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_T3_GEM_O1

const unsigned Linux::GPIO_Sysfs::pin_table[] = {
    [T3_GEM_O1_GPIO_LED_GREEN] = 380,
    [T3_GEM_O1_GPIO_LED_RED]   = 381,
    [T3_GEM_O1_GPIO2]          = 506,
    [T3_GEM_O1_GPIO3]          = 505,
    [T3_GEM_O1_GPIO4]          = 439,
    [T3_GEM_O1_GPIO5]          = 343,
    [T3_GEM_O1_GPIO6]          = 345,
    [T3_GEM_O1_GPIO7]          = 497,
    [T3_GEM_O1_GPIO8]          = 488,
    [T3_GEM_O1_GPIO9]          = 492,
    [T3_GEM_O1_GPIO10]         = 491,
    [T3_GEM_O1_GPIO11]         = 490,
    [T3_GEM_O1_GPIO12]         = 344,
    [T3_GEM_O1_GPIO13]         = 346,
    [T3_GEM_O1_GPIO14]         = 342,
    [T3_GEM_O1_GPIO15]         = 341,
    [T3_GEM_O1_GPIO16]         = 335,
    [T3_GEM_O1_GPIO17]         = 336,
    [T3_GEM_O1_GPIO18]         = 339,
    [T3_GEM_O1_GPIO19]         = 340,
    [T3_GEM_O1_GPIO20]         = 338,
    [T3_GEM_O1_GPIO21]         = 337,
    [T3_GEM_O1_GPIO22]         = 442,
    [T3_GEM_O1_GPIO23]         = 495,
    [T3_GEM_O1_GPIO24]         = 498,
    [T3_GEM_O1_GPIO25]         = 443,
    [T3_GEM_O1_GPIO26]         = 437,
    [T3_GEM_O1_GPIO27]         = 434,
};

const uint8_t Linux::GPIO_Sysfs::n_pins = _T3_GEM_O1_GPIO_MAX;

static_assert(ARRAY_SIZE(Linux::GPIO_Sysfs::pin_table) == _T3_GEM_O1_GPIO_MAX,
              "GPIO pin_table must have the same size of entries in enum gpio_t3_gem_o1");

#endif
