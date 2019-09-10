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

static uint32_t start_ms;

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

#ifdef HAL_PERIPH_NEOPIXEL_COUNT
    hal.rcout->init();
    hal.rcout->set_neopixel_num_LEDs(HAL_PERIPH_NEOPIXEL_CHAN, HAL_PERIPH_NEOPIXEL_COUNT);
#endif
    start_ms = AP_HAL::millis();
}

#if HAL_PERIPH_NEOPIXEL_COUNT == 8
/*
  rotating rainbow pattern on startup
 */
static void update_rainbow()
{
    static bool rainbow_done;
    if (rainbow_done) {
        return;
    }
    uint32_t now = AP_HAL::millis();
    if (now-start_ms > 1500) {
        rainbow_done = true;
        hal.rcout->set_neopixel_rgb_data(HAL_PERIPH_NEOPIXEL_CHAN, 0xFF, 0, 0, 0);
        hal.rcout->neopixel_send();
        return;
    }
    static uint32_t last_update_ms;
    const uint8_t step_ms = 30;
    if (now - last_update_ms < step_ms) {
        return;
    }
    const struct {
        uint8_t red;
        uint8_t green;
        uint8_t blue;
    } rgb_rainbow[] = {
        { 255, 0, 0 },
        { 255, 127, 0 },
        { 255, 255, 0 },
        { 0,   255, 0 },
        { 0,   0,   255 },
        { 75,  0,   130 },
        { 143, 0,   255 },
        { 0,   0,   0 },
    };
    last_update_ms = now;
    static uint8_t step;
    const uint8_t nsteps = ARRAY_SIZE(rgb_rainbow);
    float brightness = 0.3;
    for (uint8_t n=0; n<8; n++) {
        uint8_t i = (step + n) % nsteps;
        hal.rcout->set_neopixel_rgb_data(HAL_PERIPH_NEOPIXEL_CHAN, 1U<<n,
                                         rgb_rainbow[i].red*brightness,
                                         rgb_rainbow[i].green*brightness,
                                         rgb_rainbow[i].blue*brightness);
    }
    step++;
    hal.rcout->neopixel_send();
}
#endif



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
#ifdef HAL_PERIPH_NEOPIXEL_COUNT
        hal.rcout->set_neopixel_num_LEDs(HAL_PERIPH_NEOPIXEL_CHAN, HAL_PERIPH_NEOPIXEL_COUNT);
#endif
    }
    can_update();
    hal.scheduler->delay(1);
#if HAL_PERIPH_NEOPIXEL_COUNT == 8
    update_rainbow();
#endif
}

AP_HAL_MAIN();
