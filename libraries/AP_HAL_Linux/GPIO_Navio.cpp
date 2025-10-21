#include <AP_HAL/AP_HAL_Boards.h>

#include "GPIO_Navio.h"

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO

const unsigned Linux::GPIO_Sysfs::pin_table[] = {
    [NAVIO_GPIO_A] =       21,
    [NAVIO_GPIO_B] =       26,
    [NAVIO_GPIO_C] =       20,
    [NAVIO_GPIO_IO17] =    17,
    [NAVIO_GPIO_IO18] =    18,
    [NAVIO_GPIO_IO24] =    24,
    [NAVIO_GPIO_IO25] =    25,
    [NAVIO_GPIO_PCA_OE] =  27,
    [NAVIO_GPIO_PPM_IN] =  4,
};

const uint8_t Linux::GPIO_Sysfs::n_pins = _NAVIO_GPIO_MAX;

static_assert(ARRAY_SIZE(Linux::GPIO_Sysfs::pin_table) == _NAVIO_GPIO_MAX,
              "GPIO pin_table must have the same size of entries in enum gpio_navio");

#endif
