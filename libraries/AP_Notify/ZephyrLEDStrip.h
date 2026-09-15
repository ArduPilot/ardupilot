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
/*
  board RGB LED driven through Zephyr's led_strip API (devicetree alias
  "led-strip") - first user: an ESP32-C6 board's onboard SK6805 behind the
  ws2812-spi driver.
 */
#pragma once

#include "AP_Notify_config.h"

#if AP_NOTIFY_ZEPHYR_LED_STRIP_ENABLED

#include "RGBLed.h"

class ZephyrLEDStrip : public RGBLed {
public:
    ZephyrLEDStrip();

protected:
    bool init(void) override;
    bool hw_set_rgb(uint8_t r, uint8_t g, uint8_t b) override;

private:
    const struct device *strip;
};

#endif  // AP_NOTIFY_ZEPHYR_LED_STRIP_ENABLED
