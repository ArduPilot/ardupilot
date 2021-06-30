#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_FlashIface/AP_FlashIface.h>
#include <stdio.h>


const AP_HAL::HAL& hal = AP_HAL::get_HAL();

AP_FlashIface_JEDEC jedec_dev;

void setup();
void loop();

static AP_BoardConfig board_config;

static UNUSED_FUNCTION void test_page_program()
{
    uint8_t *data = new uint8_t[jedec_dev.get_page_size()];
    if (data == nullptr) {
        hal.console->printf("Failed to allocate data for program");
    }
    uint8_t *rdata = new uint8_t[jedec_dev.get_page_size()];
    if (rdata == nullptr) {
        hal.console->printf("Failed to allocate data for read");
    }

    // fill program data with its own adress
    for (uint32_t i = 0; i < jedec_dev.get_page_size(); i++) {
        data[i] = i;
    }
    hal.console->printf("Writing Page #1\n");
    uint32_t delay_us, timeout_us;
    uint64_t start_time_us = AP_HAL::micros64();
    if (!jedec_dev.start_program_page(0, data, delay_us, timeout_us)) {
        hal.console->printf("Page write command failed\n");
        return;
    }
    while (true) {
        hal.scheduler->delay_microseconds(delay_us);
        if (AP_HAL::micros64() > (start_time_us+delay_us)) {
            if (!jedec_dev.is_device_busy()) {
                hal.console->printf("Page Program Successful, elapsed %lld us\n", AP_HAL::micros64() - start_time_us);
                break;
            } else {
                hal.console->printf("Typical page program time reached, Still Busy?!\n");
            }
        }
        if (AP_HAL::micros64() > (start_time_us+timeout_us)) {
            hal.console->printf("Page Program Timed out, elapsed %lld us\n", AP_HAL::micros64() - start_time_us);
            return;
        }
    }
    if (!jedec_dev.read(0, rdata, jedec_dev.get_page_size())) {
        hal.console->printf("Failed to read Flash page\n");
    } else {
        if (memcmp(data, rdata, jedec_dev.get_page_size()) != 0) {
            hal.console->printf("Program Data Mismatch!\n");
        } else {
            hal.console->printf("Program Data Verified Good!\n");
        }
    }
}

static UNUSED_FUNCTION void test_sector_erase()
{
    uint32_t delay_ms, timeout_ms;
    if (!jedec_dev.start_sector_erase(0, delay_ms, timeout_ms)) { // erase first sector
        hal.console->printf("Sector erase command failed\n");
        return;
    }
    uint32_t erase_start = AP_HAL::millis();
    uint32_t next_check_ms = 0;
    hal.console->printf("Erasing Sector #1 ");
    while (true) {
        if (AP_HAL::millis() > next_check_ms) {
            hal.console->printf("\n");
            if (!jedec_dev.is_device_busy()) {
                if (next_check_ms == 0) {
                    hal.console->printf("Sector Erase happened too fast\n");
                    return;
                }
                hal.console->printf("Sector Erase Successful, elapsed %ld ms\n", AP_HAL::millis() - erase_start);
                break;
            } else {
                hal.console->printf("Still busy erasing, elapsed %ld ms\n", AP_HAL::millis() - erase_start);
            }
            if ((AP_HAL::millis() - erase_start) > timeout_ms) {
                hal.console->printf("Sector Erase Timed Out, elapsed %ld ms\n", AP_HAL::millis() - erase_start);
                return;
            }
            next_check_ms = erase_start+delay_ms;
        }
        hal.scheduler->delay((delay_ms/100) + 10);
        hal.console->printf("*");
    }
    if (!jedec_dev.verify_sector_erase(0)) {
        hal.console->printf("Erase Verification Failed!\n");
    } else {
        hal.console->printf("Erase Verification Successful!\n");
    }
}

static UNUSED_FUNCTION void test_mass_erase()
{
    uint32_t delay_ms, timeout_ms;
    if (!jedec_dev.start_mass_erase(delay_ms, timeout_ms)) {
        hal.console->printf("Mass erase command failed\n");
        return;
    }
    uint32_t erase_start = AP_HAL::millis();
    uint32_t next_check_ms = 0;
    hal.console->printf("Mass Erasing ");
    while (true) {
        if (AP_HAL::millis() > next_check_ms) {
            hal.console->printf("\n");
            if (!jedec_dev.is_device_busy()) {
                if (next_check_ms == 0) {
                    hal.console->printf("Sector Erase happened too fast\n");
                    return;
                }
                hal.console->printf("Mass Erase Successful, elapsed %ld ms\n", AP_HAL::millis() - erase_start);
                return;
            } else {
                hal.console->printf("Still busy erasing, elapsed %ld ms\n", AP_HAL::millis() - erase_start);
            }
            if ((AP_HAL::millis() - erase_start) > timeout_ms) {
                hal.console->printf("Mass Erase Timed Out, elapsed %ld ms\n", AP_HAL::millis() - erase_start);
                return;
            }
            next_check_ms = erase_start+delay_ms;
        }
        hal.scheduler->delay(delay_ms/100);
        hal.console->printf("*");
    }
}


void setup()
{
    board_config.init();
}

void loop()
{
    // Start on user input
    while (!hal.console->available()) {
        hal.scheduler->delay(100);
    }
    hal.console->printf("\n\n******************Starting Test********************\n");
    jedec_dev.init();
    test_page_program();
    test_sector_erase();
    // test_mass_erase();
    hal.scheduler->delay(1000);
}

AP_HAL_MAIN();
