#include <AP_Common/AP_Common.h>
#include "GPIO_Imx.h"
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_IMX
const unsigned Linux::GPIO_Sysfs::pin_table[] = {
    [IMX_GPIO_A] =       198,
    [IMX_GPIO_B] =       199,
    [IMX_GPIO_C] =       193,
};
const uint8_t Linux::GPIO_Sysfs::n_pins = _IMX_GPIO_MAX;
static_assert(ARRAY_SIZE(Linux::GPIO_Sysfs::pin_table) == _IMX_GPIO_MAX,
              "GPIO pin_table must have the same size of entries in enum gpio_minnow");
#endif
