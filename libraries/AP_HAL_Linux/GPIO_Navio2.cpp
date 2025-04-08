#include <AP_HAL/AP_HAL_Boards.h>

#include "GPIO_Navio2.h"

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO2

const unsigned Linux::GPIO_Sysfs::pin_table[] = {
    [NAVIO2_GPIO_PWM1] =    500,
    [NAVIO2_GPIO_PWM2] =    501,
    [NAVIO2_GPIO_PWM3] =    502,
    [NAVIO2_GPIO_PWM4] =    503,
    [NAVIO2_GPIO_PWM5] =    504,
    [NAVIO2_GPIO_PWM6] =    505,
    [NAVIO2_GPIO_PWM7] =    506,
    [NAVIO2_GPIO_PWM8] =    507,
    [NAVIO2_GPIO_PWM9] =    508,
    [NAVIO2_GPIO_PWM10] =   509,
    [NAVIO2_GPIO_PWM11] =   510,
    [NAVIO2_GPIO_PWM12] =   511,
    [NAVIO2_GPIO_PWM13] =   512,
    [NAVIO2_GPIO_PWM14] =   513,
    [NAVIO2_GPIO_IO17] =    17,
    [NAVIO2_GPIO_IO18] =    18,
    [NAVIO2_GPIO_RED] =    4,
    [NAVIO2_GPIO_GREEN] =  27,
    [NAVIO2_GPIO_BLUE] =   6,
};

const uint8_t Linux::GPIO_Sysfs::n_pins = _NAVIO2_GPIO_MAX;

static_assert(ARRAY_SIZE(Linux::GPIO_Sysfs::pin_table) == _NAVIO2_GPIO_MAX,
              "GPIO pin_table must have the same size of entries in enum gpio_navio2");

#endif
