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

#include <AP_Common/AP_Common.h>

#if AP_NOTIFY_PROFILED_ENABLED
#include "SerialLED.h"

class ProfiLED: public SerialLED {
public:
    ProfiLED();

    uint16_t init_ports() override;

};
#endif  // AP_NOTIFY_PROFILED_ENABLED

#if AP_NOTIFY_PROFILED_SPI_ENABLED
#include <AP_HAL/SPIDevice.h>
#include "RGBLed.h"

class ProfiLED_SPI: public RGBLed {
public:
    ProfiLED_SPI();

    void rgb_set_id(uint8_t r, uint8_t g, uint8_t b, uint8_t id) override;
    bool init(void) override;

protected:

    bool hw_set_rgb(uint8_t r, uint8_t g, uint8_t b) override;

private:
    uint16_t num_leds;
    struct RGB {
        uint8_t b;
        uint8_t r;
        uint8_t g;
    } *rgb;

    bool _need_update;

    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev;
    void _timer();
    // perdiodic tick to re-init
    uint32_t    _last_init_ms;
    void update_led_strip();
};


#endif  // AP_NOTIFY_PROFILED_SPI_ENABLED
