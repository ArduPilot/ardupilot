#include <AP_Common/AP_Common.h>

#include "GPIO_Minnow.h"

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_MINLURE

const unsigned Linux::GPIO_Sysfs::pin_table[] = {
    [MINNOW_GPIO_SPI_CS] =    476,
    [MINNOW_GPIO_SPI_MISO] =  477,
    [MINNOW_GPIO_SPI_MOSI] =  478,
    [MINNOW_GPIO_SPI_CLK] =   479,
    [MINNOW_GPIO_I2C_SCL] =   499,
    [MINNOW_GPIO_I2C_SDA] =   498,
    [MINNOW_GPIO_UART2_TXD] = 485,
    [MINNOW_GPIO_UART2_RXD] = 484,
    [MINNOW_GPIO_S5_0] =      338,
    [MINNOW_GPIO_S5_1] =      339,
    [MINNOW_GPIO_S5_2] =      340,
    [MINNOW_GPIO_UART1_TXD] = 481,
    [MINNOW_GPIO_UART1_RXD] = 480,
    [MINNOW_GPIO_UART1_CTS] = 483,
    [MINNOW_GPIO_UART1_RTS] = 482,
    [MINNOW_GPIO_I2S_CLK] =   472,
    [MINNOW_GPIO_I2S_FRM] =   473,
    [MINNOW_GPIO_I2S_DO] =    475,
    [MINNOW_GPIO_I2S_DI] =    474,
    [MINNOW_GPIO_PWM0] =      504,
    [MINNOW_GPIO_PWM1] =      505,
    [MINNOW_GPIO_IBL_8254] =  464,
};

const uint8_t Linux::GPIO_Sysfs::n_pins = _MINNOW_GPIO_MAX;

static_assert(ARRAY_SIZE(Linux::GPIO_Sysfs::pin_table) == _MINNOW_GPIO_MAX,
              "GPIO pin_table must have the same size of entries in enum gpio_minnow");

#endif
