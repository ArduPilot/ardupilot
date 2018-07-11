#pragma once

#include <stdint.h>
#include "AP_HAL_Linux.h"

#define LOW                 0
#define HIGH                1

#define PAGE_SIZE           (4*1024)
#define BLOCK_SIZE          (4*1024)

// Raspberry Pi GPIO mapping
#define RPI_GPIO_2    2    // Pin 3
#define RPI_GPIO_3    3    // Pin 5
#define RPI_GPIO_4    4    // Pin 7
#define RPI_GPIO_5    5    // Pin 29
#define RPI_GPIO_6    6    // Pin 31
#define RPI_GPIO_7    7    // Pin 26
#define RPI_GPIO_8    8    // Pin 24
#define RPI_GPIO_9    9    // Pin 21
#define RPI_GPIO_10   10   // Pin 19
#define RPI_GPIO_11   11   // Pin 23
#define RPI_GPIO_12   12   // Pin 32
#define RPI_GPIO_13   13   // Pin 33
#define RPI_GPIO_14   14   // Pin 8
#define RPI_GPIO_15   15   // Pin 10
#define RPI_GPIO_16   16   // Pin 36
#define RPI_GPIO_17   17   // Pin 11
#define RPI_GPIO_18   18   // Pin 12
#define RPI_GPIO_19   19   // Pin 35
#define RPI_GPIO_20   20   // Pin 38
#define RPI_GPIO_21   21   // Pin 40
#define RPI_GPIO_22   22   // Pin 15
#define RPI_GPIO_23   23   // Pin 16
#define RPI_GPIO_24   24   // Pin 18
#define RPI_GPIO_25   25   // Pin 22
#define RPI_GPIO_26   26   // Pin 37
#define RPI_GPIO_27   27   // Pin 13
#define RPI_GPIO_28   28   // Pin 3
#define RPI_GPIO_29   29   // Pin 4
#define RPI_GPIO_30   30   // Pin 5
#define RPI_GPIO_31   31   // Pin 6

namespace Linux {

class GPIO_RPI : public AP_HAL::GPIO {
public:
    GPIO_RPI();
    void    init();
    void    pinMode(uint8_t pin, uint8_t output);
    void    pinMode(uint8_t pin, uint8_t output, uint8_t alt);
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

private:
    volatile uint32_t *_gpio;
};

}
