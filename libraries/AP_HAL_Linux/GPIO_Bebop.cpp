#include <AP_HAL/AP_HAL_Boards.h>

#if HAL_LINUX_GPIO_BEBOP_ENABLED

#include "GPIO_Bebop.h"

const unsigned Linux::GPIO_Sysfs::pin_table[] = {
    [BEBOP_GPIO_CAMV_NRST] = 129,
    [LINUX_GPIO_ULTRASOUND_VOLTAGE] = 200,
};

const uint8_t Linux::GPIO_Sysfs::n_pins = _BEBOP_GPIO_MAX;

static_assert(ARRAY_SIZE(Linux::GPIO_Sysfs::pin_table) == _BEBOP_GPIO_MAX,
              "GPIO pin_table must have the same size of entries in enum gpio_bebop");

#endif  // HAL_LINUX_GPIO_BEBOP_ENABLED
