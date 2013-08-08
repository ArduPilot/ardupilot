/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_HAL_BOARDLED_H__
#define __AP_HAL_BOARDLED_H__

#include <AP_Common.h>
#include <AP_HAL.h>
#include <AP_Notify.h>

#define HIGH 1
#define LOW 0

#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
 # define HAL_GPIO_A_LED_PIN        37
 # define HAL_GPIO_B_LED_PIN        36
 # define HAL_GPIO_C_LED_PIN        35
 # define HAL_GPIO_LED_ON           HIGH
 # define HAL_GPIO_LED_OFF          LOW
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM2
 # define HAL_GPIO_A_LED_PIN        27
 # define HAL_GPIO_B_LED_PIN        26
 # define HAL_GPIO_C_LED_PIN        25
 # define HAL_GPIO_LED_ON           LOW
 # define HAL_GPIO_LED_OFF          HIGH
#elif CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
 # define HAL_GPIO_A_LED_PIN        27
 # define HAL_GPIO_B_LED_PIN        26
 # define HAL_GPIO_C_LED_PIN        25
 # define HAL_GPIO_LED_ON           LOW
 # define HAL_GPIO_LED_OFF          HIGH
#elif CONFIG_HAL_BOARD == HAL_BOARD_PX4
 # define HAL_GPIO_A_LED_PIN        27
 # define HAL_GPIO_B_LED_PIN        26
 # define HAL_GPIO_C_LED_PIN        25
 # define HAL_GPIO_LED_ON           LOW
 # define HAL_GPIO_LED_OFF          HIGH
#elif CONFIG_HAL_BOARD == HAL_BOARD_SMACCM
// XXX these are just copied, may not make sense
 # define HAL_GPIO_A_LED_PIN        27
 # define HAL_GPIO_B_LED_PIN        26
 # define HAL_GPIO_C_LED_PIN        25
 # define HAL_GPIO_LED_ON           LOW
 # define HAL_GPIO_LED_OFF          HIGH
#endif

class AP_BoardLED
{
public:
    // initialise the LED driver
    void init(void);

private:
    // private methods
    static void _update(uint32_t now);

    // private member variables
    static uint16_t _counter;
};

#endif // __AP_HAL_BOARDLED_H__
