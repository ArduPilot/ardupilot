
#ifndef __AP_HAL_LINUX_GPIO_BBB_H__
#define __AP_HAL_LINUX_GPIO_BBB_H__

#include <AP_HAL_Linux.h>

#define SYSFS_GPIO_DIR "/sys/class/gpio"

#define GPIO0_BASE 0x44E07000
#define GPIO1_BASE 0x4804C000
#define GPIO2_BASE 0x481AC000
#define GPIO3_BASE 0x481AE000

#define GPIO_SIZE  0x00000FFF

// OE: 0 is output, 1 is input
#define GPIO_OE    0x14d
#define GPIO_IN    0x14e
#define GPIO_OUT   0x14f

#define LED_AMBER       117
#define LED_BLUE        48
#define LED_SAFETY      61
#define SAFETY_SWITCH   116
#define LOW             0
#define HIGH            1

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLE || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI
#define LINUX_GPIO_NUM_BANKS 4
#else
// disable GPIO
#define LINUX_GPIO_NUM_BANKS 0
#endif

// BeagleBone Black GPIO mappings
#define BBB_USR0 53
#define BBB_USR1 54
#define BBB_USR2 55
#define BBB_USR3 56
#define BBB_P8_3 38
#define BBB_P8_4 39
#define BBB_P8_5 34
#define BBB_P8_6 35
#define BBB_P8_7 66
#define BBB_P8_8 67
#define BBB_P8_9 69
#define BBB_P8_10 68
#define BBB_P8_11 45
#define BBB_P8_12 44
#define BBB_P8_13 23
#define BBB_P8_14 26
#define BBB_P8_15 47
#define BBB_P8_16 46
#define BBB_P8_17 27
#define BBB_P8_18 65
#define BBB_P8_19 22
#define BBB_P8_20 63
#define BBB_P8_21 62
#define BBB_P8_22 37
#define BBB_P8_23 36
#define BBB_P8_24 33
#define BBB_P8_25 32
#define BBB_P8_26 61
#define BBB_P8_27 86
#define BBB_P8_28 88
#define BBB_P8_29 87
#define BBB_P8_30 89
#define BBB_P8_31 10
#define BBB_P8_32 11
#define BBB_P8_33 9
#define BBB_P8_34 81
#define BBB_P8_35 8
#define BBB_P8_36 80
#define BBB_P8_37 78
#define BBB_P8_38 79
#define BBB_P8_39 76
#define BBB_P8_40 77
#define BBB_P8_41 74
#define BBB_P8_42 75
#define BBB_P8_43 72
#define BBB_P8_44 73
#define BBB_P8_45 70
#define BBB_P8_46 71
#define BBB_P9_11 30
#define BBB_P9_12 60
#define BBB_P9_13 31
#define BBB_P9_14 50
#define BBB_P9_15 48
#define BBB_P9_16 51
#define BBB_P9_17 5
#define BBB_P9_18 4
#define BBB_P9_19 13
#define BBB_P9_20 12
#define BBB_P9_21 3
#define BBB_P9_22 2
#define BBB_P9_23 49
#define BBB_P9_24 15
#define BBB_P9_25 117
#define BBB_P9_26 14
#define BBB_P9_27 115
#define BBB_P9_28 113
#define BBB_P9_29 111
#define BBB_P9_30 112
#define BBB_P9_31 110
#define BBB_P9_41 20
#define BBB_P9_42 7

class Linux::LinuxGPIO_BBB : public AP_HAL::GPIO {
private:
    struct GPIO {
        volatile uint32_t *base;
        volatile uint32_t *oe;
        volatile uint32_t *in;
        volatile uint32_t *out;
     } gpio_bank[LINUX_GPIO_NUM_BANKS];

public:
    LinuxGPIO_BBB();
    void    init();
    void    pinMode(uint8_t pin, uint8_t output);
    int8_t  analogPinToDigitalPin(uint8_t pin);
    uint8_t read(uint8_t pin);
    void    write(uint8_t pin, uint8_t value);
    void    toggle(uint8_t pin);

    /* Alternative interface: */
    AP_HAL::DigitalSource* channel(uint16_t n);

    /* Interrupt interface: */
    bool    attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p,
            uint8_t mode);

    /* return true if USB cable is connected */
    bool    usb_connected(void);
};

#endif // __AP_HAL_LINUX_GPIO_BBB_H__
