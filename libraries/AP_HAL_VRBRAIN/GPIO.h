/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_HAL_VRBRAIN_GPIO_H__
#define __AP_HAL_VRBRAIN_GPIO_H__

#include "AP_HAL_VRBRAIN.h"

 # define HAL_GPIO_A_LED_PIN        25
 # define HAL_GPIO_B_LED_PIN        26
 # define HAL_GPIO_C_LED_PIN        27
 # define EXTERNAL_LED_GPS          28
 # define EXTERNAL_LED_ARMED        29
 # define EXTERNAL_LED_MOTOR1       30
 # define EXTERNAL_LED_MOTOR2       31
 # define BUZZER_PIN                32
 # define EXTERNAL_RELAY1_PIN       33
 # define EXTERNAL_RELAY2_PIN       34
 # define HAL_GPIO_LED_ON           HIGH
 # define HAL_GPIO_LED_OFF          LOW

class VRBRAIN::VRBRAINGPIO : public AP_HAL::GPIO {
public:
	VRBRAINGPIO();
    void    init();
    void    pinMode(uint8_t pin, uint8_t output);
    int8_t  analogPinToDigitalPin(uint8_t pin);
    uint8_t read(uint8_t pin);
    void    write(uint8_t pin, uint8_t value);
    void    toggle(uint8_t pin);

    /* Alternative interface: */
    AP_HAL::DigitalSource* channel(uint16_t n);

    /* Interrupt interface: */
    bool attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p, uint8_t mode);

    /* return true if USB cable is connected */
    bool usb_connected(void);

private:
    int _led_fd;
    int _buzzer_fd;
};

class VRBRAIN::VRBRAINDigitalSource : public AP_HAL::DigitalSource {
public:
	VRBRAINDigitalSource(uint8_t v);
    void    mode(uint8_t output);
    uint8_t read();
    void    write(uint8_t value);
    void    toggle();
private:
    uint8_t _v;
};

#endif // __AP_HAL_VRBRAIN_GPIO_H__
