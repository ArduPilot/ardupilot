#pragma once

#include "AP_HAL_PX4.h"

#define PX4_GPIO_PIEZO_PIN              110
#define PX4_GPIO_EXT_FMU_RELAY1_PIN     111
#define PX4_GPIO_EXT_FMU_RELAY2_PIN     112
#define PX4_GPIO_EXT_IO_RELAY1_PIN      113
#define PX4_GPIO_EXT_IO_RELAY2_PIN      114
#define PX4_GPIO_EXT_IO_ACC1_PIN        115
#define PX4_GPIO_EXT_IO_ACC2_PIN        116

/*
  start servo channels used as GPIO at 50. Pin 50 is
  the first FMU servo pin
 */
#define PX4_GPIO_FMU_SERVO_PIN(n)       (n+50)

#ifndef HAL_GPIO_A_LED_PIN
#define HAL_GPIO_A_LED_PIN        27
#endif
#ifndef HAL_GPIO_B_LED_PIN
#define HAL_GPIO_B_LED_PIN        26
#endif
#ifndef HAL_GPIO_C_LED_PIN
#define HAL_GPIO_C_LED_PIN        25
#endif
#ifndef HAL_GPIO_LED_ON
#define HAL_GPIO_LED_ON           0
#endif
#ifndef HAL_GPIO_LED_OFF
#define HAL_GPIO_LED_OFF          1
#endif

class PX4::PX4GPIO : public AP_HAL::GPIO {
public:
    PX4GPIO();
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
    int _gpio_fmu_fd = -1;
    int _gpio_io_fd = -1;
    bool _usb_connected = false;
};

class PX4::PX4DigitalSource : public AP_HAL::DigitalSource {
public:
    PX4DigitalSource(uint8_t v);
    void    mode(uint8_t output);
    uint8_t read();
    void    write(uint8_t value);
    void    toggle();
private:
    uint8_t _v;
};
