#include <AP_HAL/AP_HAL_Boards.h>

#if HAL_LINUX_GPIO_DISCO_ENABLED

#include "GPIO_Disco.h"

const unsigned Linux::GPIO_Sysfs::pin_table[] = {
    [DISCO_GPIO_MPU6050_DRDY] = 91,
    [LINUX_GPIO_ULTRASOUND_VOLTAGE] = 200,
};

const uint8_t Linux::GPIO_Sysfs::n_pins = _DISCO_GPIO_MAX;

static_assert(ARRAY_SIZE(Linux::GPIO_Sysfs::pin_table) == _DISCO_GPIO_MAX,
              "GPIO pin_table must have the same size of entries in enum gpio_disco");

#endif  // HAL_LINUX_GPIO_DISCO_ENABLED
