#pragma once

#include "AP_HAL_VRBRAIN.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
 # define HAL_GPIO_A_LED_PIN        25
 # define HAL_GPIO_B_LED_PIN        26
 # define HAL_GPIO_C_LED_PIN        27
 # define EXTERNAL_LED_GPS          28
 # define EXTERNAL_LED_ARMED        29
 # define EXTERNAL_LED_MOTOR1       30
 # define EXTERNAL_LED_MOTOR2       31

 # define EXTERNAL_RELAY1_PIN       34
 # define EXTERNAL_RELAY2_PIN       33

 # define HAL_GPIO_LED_ON           HIGH
 # define HAL_GPIO_LED_OFF          LOW
#endif

class VRBRAIN::VRBRAINGPIO : public AP_HAL::GPIO {
public:
	VRBRAINGPIO();
    void    init() override;
    void    pinMode(uint8_t pin, uint8_t output) override;
    uint8_t read(uint8_t pin) override;
    void    write(uint8_t pin, uint8_t value) override;
    void    toggle(uint8_t pin) override;

    /* Alternative interface: */
    AP_HAL::DigitalSource* channel(uint16_t n) override;

    /* Interrupt interface: */
    bool attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p, uint8_t mode) override;

    /* return true if USB cable is connected */
    bool usb_connected(void) override;

    // used by UART code to avoid a hw bug in the AUAV-X2
    void set_usb_connected(void) { _usb_connected = true; }

private:
    int _led_fd = -1;
    int _tone_alarm_fd = -1;
    int _gpio_fmu_fd = -1;

    bool _usb_connected = false;
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
