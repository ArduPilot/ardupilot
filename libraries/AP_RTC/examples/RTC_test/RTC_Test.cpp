//
// Simple test for the AP_RTC class
//

#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <GCS_MAVLink/GCS_Dummy.h>
#include <AP_Logger/AP_Logger.h>

#include <stdio.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static AP_BoardConfig board_config;
static AP_SerialManager serial_manager;
AP_Int32 logger_bitmask;
static AP_Logger logger;
static AP_RTC _rtc;

void setup(void)
{
    board_config.init();
    serial_manager.init();
}

void failed(const char *fmt_in, ...) FMT_PRINTF(1, 2);

void failed(const char *fmt_in, ...)
{
    va_list arg_list;

    char fmt[512];
    snprintf(fmt, ARRAY_SIZE(fmt), "### FAILED: %s\n\n", fmt_in);

#pragma GCC diagnostic ignored "-Wvarargs"
    va_start(arg_list, fmt);
    hal.console->vprintf(fmt, arg_list);
    va_end(arg_list);

    hal.scheduler->delay(2000);
}

void loop(void)
{
    static uint32_t last_run_ms;
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_run_ms < 2345) {
        return;
    }
    last_run_ms = now_ms;

    uint32_t run_counter = 0;
    AP_RTC &rtc = AP::rtc();
    // kinda_now is a time around 11:30am in Australia
    uint64_t kinda_now = 1565918797000000;
    rtc.set_utc_usec(kinda_now, AP_RTC::SOURCE_GPS);
    hal.console->printf("%s: Test run %u\n", __FILE__, (unsigned)run_counter++);
    uint64_t now = 0;
    if (!rtc.get_utc_usec(now)) {
        failed("get_utc_usec failed");
        return;
    }
    hal.console->printf("Now=%llu\n", (long long unsigned)now);
    if (now < kinda_now) {
        failed("Universe time going backwards.  Be afraid.");
        return;
    }
    { // generally make sure time is moving forward / initial time
        // offset looks right
        uint8_t hour, min, sec;
        uint16_t ms;
        if (!rtc.get_system_clock_utc(hour, min, sec, ms)) {
            failed("Failed to get hour/min/sec/ms");
            return;
        }
        if (run_counter == 0) {
            if (hour != 1 || min != 26) {
                failed("Unexpected hour/min");
                return;
            }
        }
        hal.console->printf("hour=%u min=%u sec=%u ms=%u\n", (unsigned)hour, (unsigned)min, (unsigned)sec, (unsigned)ms);
        uint32_t delta;
        // we make the assumption here that no more than a
        // millisecond can elapse between getting the system clock
        // above and these tests.
        delta = rtc.get_time_utc(hour, min, sec, ms + 1);
        if (delta != 1 && delta != 0) {
            failed("millisecond delta fail");
        }
        delta = rtc.get_time_utc(hour, min, sec + 1, ms);
        if (delta != 1000 && delta != 999) {
            failed("second delta fail");
        }
        delta = rtc.get_time_utc(hour, min + 1, sec, ms);
        if (delta != 60000 && delta != 59999) {
            failed("minute delta fail");
        }
        delta = rtc.get_time_utc(hour + 1, min, sec, ms);
        if (delta != 3600000 && delta != 3599999) {
            failed("hour delta fail");
        }
        if (min > 1) {
            delta = rtc.get_time_utc(hour, min-1, sec, ms);
            if (delta != 86340000 && delta != 86339999) {
                hal.console->printf("got: %u\n", (unsigned)rtc.get_time_utc(hour, min-1, sec, ms));
                failed("negative minute delta fail");
            }
        }
    }

    hal.console->printf("Test complete.\n\n");
}

GCS_Dummy _gcs;

AP_HAL_MAIN();
