/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 * Code by Andrew Tridgell and Siddharth Bharat Purohit
 */
#pragma once

#include "AP_HAL_ChibiOS.h"

#ifndef HAL_GPIO_LED_ON
#define HAL_GPIO_LED_ON 0
#endif

#ifndef HAL_GPIO_LED_OFF
#define HAL_GPIO_LED_OFF 1
#endif

class ChibiOS::GPIO : public AP_HAL::GPIO {
public:
    GPIO();
    void    init();
    void    pinMode(uint8_t pin, uint8_t output);
    uint8_t read(uint8_t pin);
    void    write(uint8_t pin, uint8_t value);
    void    toggle(uint8_t pin);

    /* Alternative interface: */
    AP_HAL::DigitalSource* channel(uint16_t n);

    /* Interrupt interface: */
    bool    attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p,
            uint8_t mode);

    /* return true if USB cable is connected */
    bool    usb_connected(void) override;

    void set_usb_connected() { _usb_connected = true; }

    /* attach interrupt via ioline_t */
    bool _attach_interrupt(ioline_t line, AP_HAL::Proc p, uint8_t mode);
    
private:
    bool _usb_connected;
    bool _ext_started;
};

class ChibiOS::DigitalSource : public AP_HAL::DigitalSource {
public:
    DigitalSource(ioline_t line);
    void    mode(uint8_t output);
    uint8_t read();
    void    write(uint8_t value);
    void    toggle();
private:
    ioline_t line;
};
