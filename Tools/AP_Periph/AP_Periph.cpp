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
#include <AP_HAL_ChibiOS/hwdef/common/watchdog.h>

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

/*
  declare constant app_descriptor in flash
 */
const struct app_descriptor app_descriptor __attribute__((section(".app_descriptor")));

void AP_Periph_FW::init()
{
    // always run with watchdog enabled. This should have already been
    // setup by the bootloader, but if not then enable now
    stm32_watchdog_init();

    stm32_watchdog_pat();

    hal.uartA->begin(AP_SERIALMANAGER_CONSOLE_BAUD, 32, 32);
    hal.uartB->begin(115200, 128, 256);

    load_parameters();

    stm32_watchdog_pat();

    can_start();

    serial_manager.init();

    stm32_watchdog_pat();

#ifdef HAL_BOARD_AP_PERIPH_ZUBAXGNSS
    // setup remapping register for ZubaxGNSS
    uint32_t mapr = AFIO->MAPR;
    mapr &= ~AFIO_MAPR_SWJ_CFG;
    mapr |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
    AFIO->MAPR = mapr | AFIO_MAPR_CAN_REMAP_REMAP2 | AFIO_MAPR_SPI3_REMAP;
#endif

    printf("Booting %08x:%08x %u/%u len=%u 0x%08x\n",
           app_descriptor.image_crc1,
           app_descriptor.image_crc2,
           app_descriptor.version_major, app_descriptor.version_minor,
           app_descriptor.image_size,
           app_descriptor.git_hash);

    if (hal.util->was_watchdog_reset()) {
        printf("Reboot after watchdog reset\n");
    }

#ifdef HAL_PERIPH_ENABLE_GPS
    if (gps.get_type(0) != AP_GPS::GPS_Type::GPS_TYPE_NONE) {
        gps.init(serial_manager);
    }
#endif

#ifdef HAL_PERIPH_ENABLE_MAG
    if (compass.enabled()) {
        compass.init();
    }
#endif

#ifdef HAL_PERIPH_ENABLE_BARO
    baro.init();
#endif

#ifdef HAL_PERIPH_NEOPIXEL_COUNT
    hal.rcout->init();
    hal.rcout->set_serial_led_num_LEDs(HAL_PERIPH_NEOPIXEL_CHAN, AP_HAL::RCOutput::MODE_NEOPIXEL);
#endif

#ifdef HAL_PERIPH_ENABLE_ADSB
    adsb_init();
#endif

#ifdef HAL_PERIPH_ENABLE_AIRSPEED
    if (airspeed.enabled()) {
        airspeed.init();
    }
#endif

#ifdef HAL_PERIPH_ENABLE_RANGEFINDER
    if (rangefinder.get_type(0) != RangeFinder::Type::NONE) {
        const uint8_t sernum = 3; // uartB
        hal.uartB->begin(g.rangefinder_baud);
        serial_manager.set_protocol_and_baud(sernum, AP_SerialManager::SerialProtocol_Rangefinder, g.rangefinder_baud);
        rangefinder.init(ROTATION_NONE);
    }
#endif

#ifdef HAL_PERIPH_ENABLE_PWM_HARDPOINT
    pwm_hardpoint_init();
#endif

#ifdef HAL_PERIPH_ENABLE_HWESC
    hwesc_telem.init(hal.uartB);
#endif
    
    start_ms = AP_HAL::millis();
}

#if defined(HAL_PERIPH_NEOPIXEL_COUNT) && HAL_PERIPH_NEOPIXEL_COUNT == 8
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
        hal.rcout->set_serial_led_rgb_data(HAL_PERIPH_NEOPIXEL_CHAN, -1, 0, 0, 0);
        hal.rcout->serial_led_send(HAL_PERIPH_NEOPIXEL_CHAN);
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
        hal.rcout->set_serial_led_rgb_data(HAL_PERIPH_NEOPIXEL_CHAN, n,
                                         rgb_rainbow[i].red*brightness,
                                         rgb_rainbow[i].green*brightness,
                                         rgb_rainbow[i].blue*brightness);
    }
    step++;
    hal.rcout->serial_led_send(HAL_PERIPH_NEOPIXEL_CHAN);
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
#ifdef HAL_PERIPH_ENABLE_RANGEFINDER
        hal.uartA->printf("RNG %u %ucm\n", rangefinder.num_sensors(), rangefinder.distance_cm_orient(ROTATION_NONE));
#endif
        hal.scheduler->delay(1);
        show_stack_usage();
#endif
#ifdef HAL_PERIPH_NEOPIXEL_COUNT
        hal.rcout->set_serial_led_num_LEDs(HAL_PERIPH_NEOPIXEL_CHAN, HAL_PERIPH_NEOPIXEL_COUNT, AP_HAL::RCOutput::MODE_NEOPIXEL);
#endif
    }
    can_update();
    hal.scheduler->delay(1);
#if defined(HAL_PERIPH_NEOPIXEL_COUNT) && HAL_PERIPH_NEOPIXEL_COUNT == 8
    update_rainbow();
#endif
#ifdef HAL_PERIPH_ENABLE_ADSB
    adsb_update();
#endif
}

AP_HAL_MAIN();
