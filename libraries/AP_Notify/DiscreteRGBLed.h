/*
 *  AP_Notify Library.
 * based upon a prototype library by David "Buzz" Bussenschutt.
 */

/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "AP_Notify_config.h"

#if AP_NOTIFY_DISCRETE_RGB_ENABLED

#include "RGBLed.h"

class DiscreteRGBLed: public RGBLed {
public:
    DiscreteRGBLed(uint16_t red, uint16_t green, uint16_t blue, bool polarity);
    bool init(void) override;

protected:
    bool hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue) override;

private:
    AP_HAL::DigitalSource *red_pin;
    AP_HAL::DigitalSource *green_pin;
    AP_HAL::DigitalSource *blue_pin;

    uint16_t red_pin_number, green_pin_number, blue_pin_number;
};

#endif  // AP_NOTIFY_DISCRETE_RGB_ENABLED
