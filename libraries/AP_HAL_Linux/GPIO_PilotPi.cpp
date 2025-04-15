#include "GPIO_PilotPi.h"

#if HAL_LINUX_GPIO_PILOTPI_ENABLED

uint8_t GPIO_PilotPi::read(uint8_t pin)
{
    static uint8_t real_pin;

    /* MainOut (raspberry pi GPIOs)? */
    if(mainOutPin(pin, &real_pin)) {
        return GPIO_RPI::read(real_pin);
    }
    /* AUXOUT? (rcout) */
    else if(auxOutPin(pin, &real_pin) && hal.rcout->supports_gpio()) {
        return (uint8_t)hal.rcout->read(real_pin);
    }
    return 0;
}

void GPIO_PilotPi::write(uint8_t pin, uint8_t value)
{
    static uint8_t real_pin;

    /* MainOut (raspberry pi GPIOs)? */
    if(mainOutPin(pin, &real_pin)) {
        GPIO_RPI::write(real_pin, value);
        return;
    }
    /* AUXOUT? (rcout) */
    else if(auxOutPin(pin, &real_pin) && hal.rcout->supports_gpio()) {
        hal.rcout->write_gpio(real_pin, value);
    }
}

void GPIO_PilotPi::pinMode(uint8_t pin, uint8_t output)
{
    static uint8_t real_pin;

    /* MainOut (raspberry pi GPIOs)? */
    if(mainOutPin(pin, &real_pin)) {
        GPIO_RPI::pinMode(real_pin, output);
    }
}

void GPIO_PilotPi::pinMode(uint8_t pin, uint8_t output, uint8_t alt)
{
    static uint8_t real_pin;

    /* MainOut (raspberry pi GPIOs)? */
    if(mainOutPin(pin, &real_pin)) {
        GPIO_RPI::pinMode(real_pin, output, alt);
    }
}

/** convert ardupilot RELAYx_PIN to raspberry pi gpio pin */
bool GPIO_PilotPi::mainOutPin(uint8_t ap_pin, uint8_t *pin) {
    if(100 >= ap_pin || ap_pin >= 200) {
        return false;
    }    
    /* GPIO available? */
    if(ap_pin-101 >= (uint8_t) sizeof(PilotPiGPIOS)) {
        return false;
    }
    
    *pin = PilotPiGPIOS[ap_pin-101];
    return true;
}

/* convert ardupilot RELAYx_PIN to RCOUT GPIO pin */
bool GPIO_PilotPi::auxOutPin(uint8_t ap_pin, uint8_t *pin) {
    if(50 > ap_pin || ap_pin > 100) {
        return false;
    }
    *pin = ap_pin-50;
    return true;
}

#endif  // HAL_LINUX_GPIO_PILOTPI_ENABLED
