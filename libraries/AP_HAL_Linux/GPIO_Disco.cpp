#include <AP_Common/AP_Common.h>

#include "GPIO_Disco.h"

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO

const unsigned Linux::GPIO_Sysfs::pin_table[] = {
    [DISCO_GPIO_MPU6050_DRDY] = 91,
};

const uint8_t Linux::GPIO_Sysfs::n_pins = _DISCO_GPIO_MAX;

static_assert(ARRAY_SIZE(Linux::GPIO_Sysfs::pin_table) == _DISCO_GPIO_MAX,
              "GPIO pin_table must have the same size of entries in enum gpio_disco");

#endif
