#include <AP_Common/AP_Common.h>

#include "GPIO_Bebop.h"

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP

const unsigned Linux::GPIO_Sysfs::pin_table[] = {
    [BEBOP_GPIO_CAMV_NRST] = 129,
};

const uint8_t Linux::GPIO_Sysfs::n_pins = _BEBOP_GPIO_MAX;

static_assert(ARRAY_SIZE(Linux::GPIO_Sysfs::pin_table) == _BEBOP_GPIO_MAX,
              "GPIO pin_table must have the same size of entries in enum gpio_bebop");

#endif
