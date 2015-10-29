#ifndef __AP_HAL_LINUX_GPIO_RPI_H__
#define __AP_HAL_LINUX_GPIO_RPI_H__

#include <stdint.h>
#include "AP_HAL_Linux.h"

#define LOW                 0
#define HIGH                1

// Raspberry Pi GPIO memory
#define BCM2708_PERI_BASE   0x20000000
#define BCM2709_PERI_BASE   0x3F000000
#define GPIO_BASE(address)  (address + 0x200000)
#define PWM_BASE(address)   (address + 0x20C000) /* PWM controller */
#define CLOCK_BASE(address) (address + 0x101000)

#define PWM_CTL  0
#define PWM_RNG1 4
#define PWM_DAT1 5

#define PWMCLK_CNTL 40
#define PWMCLK_DIV  41

#define PAGE_SIZE           (4*1024)
#define BLOCK_SIZE          (4*1024)

// GPIO setup. Always use INP_GPIO(x) before OUT_GPIO(x) or SET_GPIO_ALT(x,y)
#define GPIO_MODE_IN(g)     *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define GPIO_MODE_OUT(g)    *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define GPIO_MODE_ALT(g,a)  *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))
#define GPIO_SET_HIGH       *(gpio+7)  // sets   bits which are 1
#define GPIO_SET_LOW        *(gpio+10) // clears bits which are 1
#define GPIO_GET(g)         (*(gpio+13)&(1<<g)) // 0 if LOW, (1<<g) if HIGH

// Raspberry Pi GPIO mapping
#define RPI_GPIO_2    2    // Pin 3     SDA
#define RPI_GPIO_3    3    // Pin 5     SCL
#define RPI_GPIO_4    4    // Pin 7
#define RPI_GPIO_5    5    // Pin 29
#define RPI_GPIO_6    6    // Pin 31
#define RPI_GPIO_7    7    // Pin 26    CE1     MPU9250_CS
#define RPI_GPIO_8    8    // Pin 24    CE0     UBLOX_CS
#define RPI_GPIO_9    9    // Pin 21    MISO
#define RPI_GPIO_10   10   // Pin 19    MOSI
#define RPI_GPIO_11   11   // Pin 23    SCLK
#define RPI_GPIO_12   12   // Pin 32
#define RPI_GPIO_13   13   // Pin 33
#define RPI_GPIO_14   14   // Pin 8     TxD
#define RPI_GPIO_15   15   // Pin 10    RxD
#define RPI_GPIO_16   16   // Pin 36
#define RPI_GPIO_17   17   // Pin 11            UART_PORT_5
#define RPI_GPIO_18   18   // Pin 12            UART_PORT_4
#define RPI_GPIO_19   19   // Pin 35
#define RPI_GPIO_20   20   // Pin 38
#define RPI_GPIO_21   21   // Pin 40
#define RPI_GPIO_22   22   // Pin 15            UBLOX_PPS
#define RPI_GPIO_23   23   // Pin 16            MPU9250_DRDY
#define RPI_GPIO_24   24   // Pin 18            SPI_PORT_6
#define RPI_GPIO_25   25   // Pin 22            SPI_PORT_5
#define RPI_GPIO_26   26   // Pin 37
#define RPI_GPIO_27   27   // Pin 13
#define RPI_GPIO_28   28   // Pin 3
#define RPI_GPIO_29   29   // Pin 4
#define RPI_GPIO_30   30   // Pin 5
#define RPI_GPIO_31   31   // Pin 6

class Linux::GPIO_RPI : public AP_HAL::GPIO {
private:
    int  mem_fd;
    void *gpio_map;
    void *pwm_map;
    void *clk_map;
    volatile uint32_t *gpio;
    volatile uint32_t *pwm;
    volatile uint32_t *clk;
    void setPWM0Period(uint32_t time_us);
    void setPWM0Duty(uint8_t percent);

public:
    GPIO_RPI();
    void    init();
    void    pinMode(uint8_t pin, uint8_t output);
    void    pinMode(uint8_t pin, uint8_t output, uint8_t alt);
    int8_t  analogPinToDigitalPin(uint8_t pin);
    uint8_t read(uint8_t pin);
    void    write(uint8_t pin, uint8_t value);
    void    toggle(uint8_t pin);
    void    setPWMPeriod(uint8_t pin, uint32_t time_us);
    void    setPWMDuty(uint8_t pin, uint8_t percent);

    /* Alternative interface: */
    AP_HAL::DigitalSource* channel(uint16_t n);

    /* Interrupt interface: */
    bool    attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p,
            uint8_t mode);

    /* return true if USB cable is connected */
    bool    usb_connected(void);
};

#endif // __AP_HAL_LINUX_GPIO_RPI_H__