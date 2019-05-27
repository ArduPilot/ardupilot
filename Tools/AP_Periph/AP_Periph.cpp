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
  AP_Periph main firmware

  To flash this firmware on Linux use:

     st-flash write build/f103-periph/bin/AP_Periph.bin 0x8006000

 */
#include <AP_HAL/AP_HAL.h>
#include "AP_Periph.h"
#include "hal.h"
#include <stdio.h>
#include <AP_HAL_ChibiOS/hwdef/common/stm32_util.h>

extern const AP_HAL::HAL &hal;

AP_Periph_FW periph;

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

void setup(void)
{
    periph.init();
}

void loop(void)
{
    periph.update();
}

void AP_Periph_FW::init()
{
    hal.uartA->begin(AP_SERIALMANAGER_CONSOLE_BAUD, 32, 128);
    hal.uartB->begin(115200, 32, 128);

    load_parameters();
    can_start();

    serial_manager.init();

#ifdef HAL_PERIPH_ENABLE_GPS
    gps.init(serial_manager);
#endif

#ifdef HAL_PERIPH_ENABLE_MAG
    compass.init();
#endif

#ifdef HAL_PERIPH_ENABLE_BARO
    baro.init();
    baro.calibrate(false);
#endif
}

void AP_Periph_FW::update()
{
    static uint32_t last_led_ms;
    uint32_t now = AP_HAL::millis();
    if (now - last_led_ms > 1000) {
        last_led_ms = now;
        palToggleLine(HAL_GPIO_PIN_LED);
#if 0
#ifdef HAL_PERIPH_ENABLE_GPS
        hal.uartA->printf("GPS status: %u\n", (unsigned)gps.status());
#endif
#ifdef HAL_PERIPH_ENABLE_MAG
        const Vector3f &field = compass.get_field();
        hal.uartA->printf("MAG (%d,%d,%d)\n", int(field.x), int(field.y), int(field.z));
#endif
#ifdef HAL_PERIPH_ENABLE_BARO
        hal.uartA->printf("BARO H=%u P=%.2f T=%.2f\n", baro.healthy(), baro.get_pressure(), baro.get_temperature());
#endif
        hal.scheduler->delay(1);
        show_stack_usage();
#endif
    }
    can_update();
    hal.scheduler->delay(1);
}

AP_HAL_MAIN();
