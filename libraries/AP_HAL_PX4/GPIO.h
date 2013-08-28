/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_HAL_PX4_GPIO_H__
#define __AP_HAL_PX4_GPIO_H__

#include <AP_HAL_PX4.h>

#define PX4_GPIO_PIEZO_PIN              110
#define PX4_GPIO_EXT_FMU_RELAY1_PIN     111
#define PX4_GPIO_EXT_FMU_RELAY2_PIN     112
#define PX4_GPIO_EXT_IO_RELAY1_PIN      113
#define PX4_GPIO_EXT_IO_RELAY2_PIN      114
#define PX4_GPIO_EXT_IO_ACC1_PIN        115
#define PX4_GPIO_EXT_IO_ACC2_PIN        116

class PX4::PX4GPIO : public AP_HAL::GPIO {
public:
    PX4GPIO();
    void    init();
    void    pinMode(uint8_t pin, uint8_t output);
    int8_t  analogPinToDigitalPin(uint8_t pin);
    uint8_t read(uint8_t pin);
    void    write(uint8_t pin, uint8_t value);

    /* Alternative interface: */
    AP_HAL::DigitalSource* channel(uint16_t n);

    /* Interrupt interface: */
    bool attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p, uint8_t mode);

private:
    int _led_fd;
    int _tone_alarm_fd;
    int _gpio_fmu_fd;
    int _gpio_io_fd;
};

class PX4::PX4DigitalSource : public AP_HAL::DigitalSource {
public:
    PX4DigitalSource(uint8_t v);
    void    mode(uint8_t output);
    uint8_t read();
    void    write(uint8_t value); 
private:
    uint8_t _v;
};

#endif // __AP_HAL_PX4_GPIO_H__
