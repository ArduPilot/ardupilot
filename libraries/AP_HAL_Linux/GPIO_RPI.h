
#ifndef __AP_HAL_LINUX_GPIO_RPI_H__
#define __AP_HAL_LINUX_GPIO_RPI_H__

#include <stdint.h>
#include <AP_HAL_Linux.h>

#define LOW                 0
#define HIGH                1

// Raspberry Pi GPIO memory
#define BCM2708_PERI_BASE   0x20000000
#define GPIO_BASE           (BCM2708_PERI_BASE + 0x200000)
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
#define RPI_GPIO_4    4    // Pin 7             NAVIO_PPM_INPUT
#define RPI_GPIO_7    7    // Pin 26    CE1     NAVIO_MPU9250_CS
#define RPI_GPIO_8    8    // Pin 24    CE0     NAVIO_UBLOX_CS
#define RPI_GPIO_9    9    // Pin 21    MISO
#define RPI_GPIO_10   10   // Pin 19    MOSI
#define RPI_GPIO_11   11   // Pin 23    SCLK
#define RPI_GPIO_14   14   // Pin 8     TxD
#define RPI_GPIO_15   15   // Pin 10    RxD
#define RPI_GPIO_17   17   // Pin 11            NAVIO_UART_PORT_5
#define RPI_GPIO_18   18   // Pin 12            NAVIO_UART_PORT_4
#define RPI_GPIO_22   22   // Pin 15            NAVIO_UBLOX_PPS
#define RPI_GPIO_23   23   // Pin 16            NAVIO_MPU9250_DRDY
#define RPI_GPIO_24   24   // Pin 18            NAVIO_SPI_PORT_6
#define RPI_GPIO_25   25   // Pin 22            NAVIO_SPI_PORT_5
#define RPI_GPIO_27   27   // Pin 13            NAVIO_PCA9685_OE
#define RPI_GPIO_28   28   // Pin 3
#define RPI_GPIO_29   29   // Pin 4
#define RPI_GPIO_30   30   // Pin 5
#define RPI_GPIO_31   31   // Pin 6

class Linux::LinuxGPIO_RPI : public AP_HAL::GPIO {
private:
    int  mem_fd;
    void *gpio_map;
    volatile uint32_t *gpio;

public:
    LinuxGPIO_RPI();
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

#endif // __AP_HAL_LINUX_GPIO_RPI_H__
