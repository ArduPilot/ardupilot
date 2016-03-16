
#ifndef __AP_HAL_REVOMINI_GPIO_H__
#define __AP_HAL_REVOMINI_GPIO_H__

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_REVOMINI_Namespace.h"
#include "gpio_hal.h"

#ifndef HIGH
 #define HIGH 0x1
#endif

#ifndef LOW
 #define LOW  0x0
#endif


#if CONFIG_HAL_BOARD == HAL_BOARD_REVOMINI
 #define BUZZER_PIN 200

 # define HAL_GPIO_A_LED_PIN        36  // BLUE
 # define HAL_GPIO_B_LED_PIN        106 //  LED PA13
 # define HAL_GPIO_C_LED_PIN        105 // RED
 # define HAL_GPIO_LED_ON           LOW
 # define HAL_GPIO_LED_OFF          HIGH
#endif

class REVOMINI::REVOMINIGPIO : public AP_HAL::GPIO {
public:
    REVOMINIGPIO();
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

class REVOMINI::REVOMINIDigitalSource : public AP_HAL::DigitalSource {
public:
    REVOMINIDigitalSource(gpio_dev *device, uint8_t bit): _device(device), _bit(bit){}
    void    mode(uint8_t output);
    uint8_t read();
    void    write(uint8_t value); 
    void    toggle();
private:
    gpio_dev *_device;
    uint8_t _bit;
};



#endif // __AP_HAL_REVOMINI_GPIO_H__
