#pragma once

#include "AP_HAL_Linux.h"

#if HAL_LINUX_GPIO_PB2_ENABLED

//GPIO Addressess
#define GPIO0_BASE  0x600000
#define GPIO1_BASE  0x601000
#define GPIO_SIZE   0xffff

//Registers
//Bank0 and 1
#define GPIO_DIR01      0X4
#define GPIO_SET_DATA01 0x6
#define GPIO_OUT_DATA01 0x5
#define GPIO_CLR_DATA01 0x7
#define GPIO_IN_DATA01  0x8

//Bank2 and 3
#define GPIO_DIR23      0xE
#define GPIO_SET_DATA23 0x10
#define GPIO_OUT_DATA23 0xF
#define GPIO_CLR_DATA23 0x11
#define GPIO_IN_DATA23  0x12

//Bank 4 and 5
#define GPIO_DIR45      0x18
#define GPIO_SET_DATA45 0x1A
#define GPIO_OUT_DATA45 0x19
#define GPIO_CLR_DATA45 0x1B
#define GPIO_IN_DATA45  0x1C

#define LINUX_GPIO_NUM_BANKS 6

#define LED_AMBER       45
#define LED_BLUE        46
#define LED_SAFETY      91

// PocketBeagle 2 GPIO mappings
#define PB2_USR0 3
#define PB2_USR1 4
#define PB2_USR2 5
#define PB2_USR3 6
#define PB2_P1_2 10
#define PB2_P1_3 51
#define PB2_P1_4 12
#define PB2_P1_6 13
#define PB2_P1_8 14
#define PB2_P1_10 30
#define PB2_P1_12 8
#define PB2_P1_20 50
#define PB2_P1_26 44
#define PB2_P1_28 43
#define PB2_P1_29 62
#define PB2_P1_30 21
#define PB2_P1_31 29
#define PB2_P1_32 20
#define PB2_P1_33 29
#define PB2_P1_34 2
#define PB2_P1_35 88
#define PB2_P1_36 28
#define PB2_P2_1 86
#define PB2_P2_2 45
#define PB2_P2_3 9
#define PB2_P2_4 46
#define PB2_P2_5 24
#define PB2_P2_6 47
#define PB2_P2_7 25
#define PB2_P2_8 48
#define PB2_P2_9 22
#define PB2_P2_10 91
#define PB2_P2_11 23
#define PB2_P2_17 64
#define PB2_P2_18 53
#define PB2_P2_19 0
#define PB2_P2_20 49
#define PB2_P2_22 63
#define PB2_P2_24 51
#define PB2_P2_25 19
#define PB2_P2_27 18
#define PB2_P2_28 61
#define PB2_P2_29 17
#define PB2_P2_30 58
#define PB2_P2_31 15
#define PB2_P2_32 57
#define PB2_P2_33 52
#define PB2_P2_34 60
#define PB2_P2_35 54
#define PB2_P2_36 16

namespace Linux {

class GPIO_PB2 : public AP_HAL::GPIO {
private:
    struct GPIO {
        volatile uint32_t *gpio_base;
        volatile uint32_t *gpio_set;
        volatile uint32_t *gpio_clear;
        volatile uint32_t *gpio_out;
        volatile uint32_t *gpio_direction;
        volatile uint32_t *gpio_in;
    } gpio_bank[LINUX_GPIO_NUM_BANKS];

public:
    GPIO_PB2();
    void    init() override;
    void    pinMode(uint8_t pin, uint8_t output) override;
    uint8_t read(uint8_t pin) override;
    void    write(uint8_t pin, uint8_t value) override;
    void    toggle(uint8_t pin) override;

    /* Alternative interface: */
    AP_HAL::DigitalSource* channel(uint16_t n) override;

    /* return true if USB cable is connected */
    bool    usb_connected(void) override;
};

}
#endif  // HAL_LINUX_GPIO_PB2_ENABLED