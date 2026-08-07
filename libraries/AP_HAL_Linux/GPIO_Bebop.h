#pragma once

#if HAL_LINUX_GPIO_BEBOP_ENABLED

#include "GPIO_Sysfs.h"

enum gpio_bebop {
    BEBOP_GPIO_CAMV_NRST,
    LINUX_GPIO_ULTRASOUND_VOLTAGE,
    _BEBOP_GPIO_MAX,
};

#endif  // HAL_LINUX_GPIO_BEBOP_ENABLED
