#include <AP_HAL/AP_HAL_Boards.h>

#include "GPIO_Edge.h"

#if HAL_LINUX_GPIO_EDGE_ENABLED

const unsigned Linux::GPIO_Sysfs::pin_table[] = {
    [EDGE_GPIO_PWM1] =    500,
    [EDGE_GPIO_PWM2] =    501,
    [EDGE_GPIO_PWM3] =    502,
    [EDGE_GPIO_PWM4] =    503,
    [EDGE_GPIO_PWM5] =    504,
    [EDGE_GPIO_PWM6] =    505,
    [EDGE_GPIO_PWM7] =    506,
    [EDGE_GPIO_PWM8] =    507,
    [EDGE_GPIO_PWM9] =    508,
    [EDGE_GPIO_PWM10] =   509,
    [EDGE_GPIO_PWM11] =   510,
    [EDGE_GPIO_PWM12] =   511,
    [EDGE_GPIO_HEAT_ENABLE] = 26,
};

const uint8_t Linux::GPIO_Sysfs::n_pins = _EDGE_GPIO_MAX;

static_assert(ARRAY_SIZE(Linux::GPIO_Sysfs::pin_table) == _EDGE_GPIO_MAX,
              "GPIO pin_table must have the same size of entries in enum edge");

#endif  // HAL_LINUX_GPIO_EDGE_ENABLED
