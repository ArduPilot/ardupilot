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
#include <stdio.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include <AP_HAL_ChibiOS/hwdef/common/stm32_util.h>
#include <AP_HAL_ChibiOS/hwdef/common/watchdog.h>
#endif

extern const AP_HAL::HAL &hal;

AP_Periph_FW periph;

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
void stm32_watchdog_init() {}
void stm32_watchdog_pat() {}
#endif

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
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
const struct app_descriptor app_descriptor __attribute__((section(".app_descriptor")));
#else
const struct app_descriptor app_descriptor;
#endif

void AP_Periph_FW::init()
{
    
    // always run with watchdog enabled. This should have already been
    // setup by the bootloader, but if not then enable now
    stm32_watchdog_init();

    stm32_watchdog_pat();

    hal.serial(0)->begin(AP_SERIALMANAGER_CONSOLE_BAUD, 32, 32);
    hal.serial(3)->begin(115200, 128, 256);

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
    if (gps.get_type(0) != AP_GPS::GPS_Type::GPS_TYPE_NONE && g.gps_port >= 0) {
        serial_manager.set_protocol_and_baud(g.gps_port, AP_SerialManager::SerialProtocol_GPS, AP_SERIALMANAGER_GPS_BAUD);
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

#ifdef HAL_PERIPH_ENABLE_BATTERY
    battery.lib.init();
#endif

#if defined(HAL_PERIPH_NEOPIXEL_COUNT_WITHOUT_NOTIFY) || defined(HAL_PERIPH_ENABLE_RC_OUT)
    hal.rcout->init();
#endif

#ifdef HAL_PERIPH_NEOPIXEL_CHAN_WITHOUT_NOTIFY
    hal.rcout->set_serial_led_num_LEDs(HAL_PERIPH_NEOPIXEL_CHAN_WITHOUT_NOTIFY, AP_HAL::RCOutput::MODE_NEOPIXEL);
#endif

#ifdef HAL_PERIPH_ENABLE_RC_OUT
    rcout_init();
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
    if (rangefinder.get_type(0) != RangeFinder::Type::NONE && g.rangefinder_port >= 0) {
        auto *uart = hal.serial(g.rangefinder_port);
        if (uart != nullptr) {
            uart->begin(g.rangefinder_baud);
            serial_manager.set_protocol_and_baud(g.rangefinder_port, AP_SerialManager::SerialProtocol_Rangefinder, g.rangefinder_baud);
            rangefinder.init(ROTATION_NONE);
        }
    }
#endif

#ifdef HAL_PERIPH_ENABLE_PWM_HARDPOINT
    pwm_hardpoint_init();
#endif

#ifdef HAL_PERIPH_ENABLE_HWESC
    hwesc_telem.init(hal.serial(3));
#endif

#ifdef HAL_PERIPH_ENABLE_MSP
    if (g.msp_port >= 0) {
        msp_init(hal.serial(g.msp_port));
    }
#endif
    
#ifdef HAL_PERIPH_ENABLE_NOTIFY
    notify.init();
#endif

    start_ms = AP_HAL::native_millis();
}

#if (defined(HAL_PERIPH_NEOPIXEL_COUNT_WITHOUT_NOTIFY) && HAL_PERIPH_NEOPIXEL_COUNT_WITHOUT_NOTIFY == 8) || defined(HAL_PERIPH_ENABLE_NOTIFY)
/*
  rotating rainbow pattern on startup
 */
void AP_Periph_FW::update_rainbow()
{
#ifdef HAL_PERIPH_ENABLE_NOTIFY
    if (notify.get_led_len() != 8) {
        return;
    }
#endif
    static bool rainbow_done;
    if (rainbow_done) {
        return;
    }
    uint32_t now = AP_HAL::native_millis();
    if (now - start_ms > 1500) {
        rainbow_done = true;
#if defined (HAL_PERIPH_ENABLE_NOTIFY)
        periph.notify.handle_rgb(0, 0, 0);
#elif defined(HAL_PERIPH_NEOPIXEL_CHAN_WITHOUT_NOTIFY)
        hal.rcout->set_serial_led_rgb_data(HAL_PERIPH_NEOPIXEL_CHAN_WITHOUT_NOTIFY, -1, 0, 0, 0);
        hal.rcout->serial_led_send(HAL_PERIPH_NEOPIXEL_CHAN_WITHOUT_NOTIFY);
#endif
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
#if defined (HAL_PERIPH_ENABLE_NOTIFY)
        periph.notify.handle_rgb(
#elif defined(HAL_PERIPH_NEOPIXEL_CHAN_WITHOUT_NOTIFY)
        hal.rcout->set_serial_led_rgb_data(HAL_PERIPH_NEOPIXEL_CHAN_WITHOUT_NOTIFY, n,
#endif
                                        rgb_rainbow[i].red*brightness,
                                        rgb_rainbow[i].green*brightness,
                                        rgb_rainbow[i].blue*brightness);
    }
    step++;

#if defined(HAL_PERIPH_NEOPIXEL_CHAN_WITHOUT_NOTIFY)
    hal.rcout->serial_led_send(HAL_PERIPH_NEOPIXEL_CHAN_WITHOUT_NOTIFY);
#endif
}
#endif // HAL_PERIPH_ENABLE_NOTIFY


#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS && CH_DBG_ENABLE_STACK_CHECK == TRUE
void AP_Periph_FW::show_stack_free()
{
    const uint32_t isr_stack_size = uint32_t((const uint8_t *)&__main_stack_end__ - (const uint8_t *)&__main_stack_base__);
    can_printf("ISR %u/%u", stack_free(&__main_stack_base__), isr_stack_size);

    for (thread_t *tp = chRegFirstThread(); tp; tp = chRegNextThread(tp)) {
        uint32_t total_stack;
        if (tp->wabase == (void*)&__main_thread_stack_base__) {
            // main thread has its stack separated from the thread context
            total_stack = uint32_t((const uint8_t *)&__main_thread_stack_end__ - (const uint8_t *)&__main_thread_stack_base__);
        } else {
            // all other threads have their thread context pointer
            // above the stack top
            total_stack = uint32_t(tp) - uint32_t(tp->wabase);
        }
        can_printf("%s STACK=%u/%u\n", tp->name, stack_free(tp->wabase), total_stack);
    }
}
#endif



void AP_Periph_FW::update()
{
    static uint32_t last_led_ms;
    uint32_t now = AP_HAL::native_millis();
    if (now - last_led_ms > 1000) {
        last_led_ms = now;
#ifdef HAL_GPIO_PIN_LED
        palToggleLine(HAL_GPIO_PIN_LED);
#endif
#if 0
#ifdef HAL_PERIPH_ENABLE_GPS
        hal.serial(0)->printf("GPS status: %u\n", (unsigned)gps.status());
#endif
#ifdef HAL_PERIPH_ENABLE_MAG
        const Vector3f &field = compass.get_field();
        hal.serial(0)->printf("MAG (%d,%d,%d)\n", int(field.x), int(field.y), int(field.z));
#endif
#ifdef HAL_PERIPH_ENABLE_BARO
        hal.serial(0)->printf("BARO H=%u P=%.2f T=%.2f\n", baro.healthy(), baro.get_pressure(), baro.get_temperature());
#endif
#ifdef HAL_PERIPH_ENABLE_RANGEFINDER
        hal.serial(0)->printf("RNG %u %ucm\n", rangefinder.num_sensors(), rangefinder.distance_cm_orient(ROTATION_NONE));
#endif
        hal.scheduler->delay(1);
#endif
#ifdef HAL_PERIPH_NEOPIXEL_COUNT_WITHOUT_NOTIFY
        hal.rcout->set_serial_led_num_LEDs(HAL_PERIPH_NEOPIXEL_CHAN_WITHOUT_NOTIFY, HAL_PERIPH_NEOPIXEL_COUNT_WITHOUT_NOTIFY, AP_HAL::RCOutput::MODE_NEOPIXEL);
#endif

#ifdef HAL_PERIPH_LISTEN_FOR_SERIAL_UART_REBOOT_CMD_PORT
        check_for_serial_reboot_cmd(HAL_PERIPH_LISTEN_FOR_SERIAL_UART_REBOOT_CMD_PORT);
#endif

#ifdef HAL_PERIPH_ENABLE_RC_OUT
        rcout_init_1Hz();
#endif
    }

    static uint32_t last_error_ms;
    const auto &ierr = AP::internalerror();
    if (now - last_error_ms > 5000 && ierr.errors()) {
        // display internal errors as DEBUG every 5s
        last_error_ms = now;
        can_printf("IERR 0x%x %u", ierr.errors(), ierr.last_error_line());
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS && CH_DBG_ENABLE_STACK_CHECK == TRUE
    static uint32_t last_debug_ms;
    if (g.debug==1 && now - last_debug_ms > 5000) {
        last_debug_ms = now;
        show_stack_free();
    }
#endif
    
#ifdef HAL_PERIPH_ENABLE_BATTERY
    if (now - battery.last_read_ms >= 100) {
        // update battery at 10Hz
        battery.last_read_ms = now;
        battery.lib.read();
    }
#endif

#ifdef HAL_PERIPH_ENABLE_NOTIFY
    static uint32_t notify_last_update_ms;
    if (now - notify_last_update_ms >= 20) {
        // update notify at 50Hz
        notify_last_update_ms = now;
        notify.update();
    }
#endif

    can_update();
    hal.scheduler->delay(1);
#if (defined(HAL_PERIPH_NEOPIXEL_COUNT_WITHOUT_NOTIFY) && HAL_PERIPH_NEOPIXEL_COUNT_WITHOUT_NOTIFY == 8) || defined(HAL_PERIPH_ENABLE_NOTIFY)
    update_rainbow();
#endif
#ifdef HAL_PERIPH_ENABLE_ADSB
    adsb_update();
#endif
}

#ifdef HAL_PERIPH_LISTEN_FOR_SERIAL_UART_REBOOT_CMD_PORT
// check for uploader.py reboot command
void AP_Periph_FW::check_for_serial_reboot_cmd(const int8_t serial_index)
{
    // These are the string definitions in uploader.py
    //            NSH_INIT        = bytearray(b'\x0d\x0d\x0d')
    //            NSH_REBOOT_BL   = b"reboot -b\n"
    //            NSH_REBOOT      = b"reboot\n"

    // This is the command sequence that is sent from uploader.py
    //            self.__send(uploader.NSH_INIT)
    //            self.__send(uploader.NSH_REBOOT_BL)
    //            self.__send(uploader.NSH_INIT)
    //            self.__send(uploader.NSH_REBOOT)

    for (uint8_t i=0; i<hal.num_serial; i++) {
        if (serial_index >= 0 && serial_index != i) {
            // a specific serial port was selected but this is not it
            continue;
        }

        auto *uart = hal.serial(i);
        if (uart == nullptr || !uart->is_initialized()) {
            continue;
        }

        uint32_t available = MIN(uart->available(), 1000U);
        while (available-- > 0) {
            const char reboot_string[] = "\r\r\rreboot -b\n\r\r\rreboot\n";
            const char reboot_string_len = sizeof(reboot_string)-1; // -1 is to remove the null termination
            static uint16_t index[hal.num_serial];

            const int16_t data = uart->read();
            if (data < 0 || data > 0xff) {
                // read error
                continue;
            }
            if (index[i] >= reboot_string_len || (uint8_t)data != reboot_string[index[i]]) {
                // don't have a perfect match, start over
                index[i] = 0;
                continue;
            }
            index[i]++;
            if (index[i] == reboot_string_len) {
                // received reboot msg. Trigger a reboot and stay in the bootloader
                prepare_reboot();
                hal.scheduler->reboot(true);
            }
        }
    }
}
#endif // HAL_PERIPH_LISTEN_FOR_SERIAL_UART_REBOOT_CMD_PORT

// prepare for a safe reboot where PWMs and params are gracefully disabled
// This is copied from AP_Vehicle::reboot(bool hold_in_bootloader) minus the actual reboot
void AP_Periph_FW::prepare_reboot()
{
#ifdef HAL_PERIPH_ENABLE_RC_OUT
        // force safety on
        hal.rcout->force_safety_on();
#endif

        // flush pending parameter writes
        AP_Param::flush();

        // do not process incoming mavlink messages while we delay:
        hal.scheduler->register_delay_callback(nullptr, 5);

        // delay to give the ACK a chance to get out, the LEDs to flash,
        // the IO board safety to be forced on, the parameters to flush,
        hal.scheduler->delay(40);
}

AP_HAL_MAIN();
