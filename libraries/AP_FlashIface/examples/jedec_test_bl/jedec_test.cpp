#include <AP_HAL/AP_HAL.h>
#include <AP_FlashIface/AP_FlashIface.h>
#include <stdio.h>
#include <support.h>

AP_FlashIface_JEDEC jedec_dev;

static UNUSED_FUNCTION void test_page_program()
{
    uint8_t *data = new uint8_t[jedec_dev.get_page_size()];
    if (data == nullptr) {
        uprintf("Failed to allocate data for program");
    }
    uint8_t *rdata = new uint8_t[jedec_dev.get_page_size()];
    if (rdata == nullptr) {
        uprintf("Failed to allocate data for read");
    }

    // fill program data with its own adress
    for (uint32_t i = 0; i < jedec_dev.get_page_size(); i++) {
        data[i] = i;
    }
    uprintf("Writing Page #1\n");
    uint32_t delay_us, timeout_us;
    uint64_t start_time_us = AP_HAL::micros64();
    if (!jedec_dev.start_program_page(0, data, delay_us, timeout_us)) {
        uprintf("Page write command failed\n");
        return;
    }
    while (true) {
        chThdSleep(chTimeUS2I(delay_us));
        if (AP_HAL::micros64() > (start_time_us+delay_us)) {
            if (!jedec_dev.is_device_busy()) {
                uprintf("Page Program Successful, elapsed %ld us\n", uint32_t(AP_HAL::micros64() - start_time_us));
                break;
            } else {
                uprintf("Typical page program time reached, Still Busy?!\n");
            }
        }
        if (AP_HAL::micros64() > (start_time_us+timeout_us)) {
            uprintf("Page Program Timed out, elapsed %lld us\n", AP_HAL::micros64() - start_time_us);
            return;
        }
    }
    if (!jedec_dev.read(0, rdata, jedec_dev.get_page_size())) {
        uprintf("Failed to read Flash page\n");
    } else {
        if (memcmp(data, rdata, jedec_dev.get_page_size()) != 0) {
            uprintf("Program Data Mismatch!\n");
        } else {
            uprintf("Program Data Verified Good!\n");
        }
    }

    // Now test XIP mode here as well
    uint8_t *chip_data = nullptr;
    if (!jedec_dev.start_xip_mode((void**)&chip_data)) {
        uprintf("Failed to setup XIP mode\n");
    }
    if (chip_data == nullptr) {
        uprintf("Invalid address!\n");
    }
    // Here comes the future!
    if (memcmp(data, chip_data, jedec_dev.get_page_size()) != 0) {
        uprintf("Program Data Mismatch in XIP mode!\n");
    } else {
        uprintf("Program Data Verified Good in XIP mode!\n");
    }
    jedec_dev.stop_xip_mode();
}

static UNUSED_FUNCTION void test_sector_erase()
{
    uint32_t delay_ms, timeout_ms;
    if (!jedec_dev.start_sector_erase(0, delay_ms, timeout_ms)) { // erase first sector
        uprintf("Sector erase command failed\n");
        return;
    }
    uint32_t erase_start = AP_HAL::millis();
    uint32_t next_check_ms = 0;
    uprintf("Erasing Sector #1 ");
    while (true) {
        if (AP_HAL::millis() > next_check_ms) {
            uprintf("\n");
            if (!jedec_dev.is_device_busy()) {
                if (next_check_ms == 0) {
                    uprintf("Sector Erase happened too fast\n");
                    return;
                }
                uprintf("Sector Erase Successful, elapsed %ld ms\n", AP_HAL::millis() - erase_start);
                break;
            } else {
                uprintf("Still busy erasing, elapsed %ld ms\n", AP_HAL::millis() - erase_start);
            }
            if ((AP_HAL::millis() - erase_start) > timeout_ms) {
                uprintf("Sector Erase Timed Out, elapsed %ld ms\n", AP_HAL::millis() - erase_start);
                return;
            }
            next_check_ms = erase_start+delay_ms;
        }
        chThdSleep(chTimeMS2I((delay_ms/100) + 10));
        uprintf("*");
    }
    if (!jedec_dev.verify_sector_erase(0)) {
        uprintf("Erase Verification Failed!\n");
    } else {
        uprintf("Erase Verification Successful!\n");
    }
}

static UNUSED_FUNCTION void test_mass_erase()
{
    uint32_t delay_ms, timeout_ms;
    if (!jedec_dev.start_mass_erase(delay_ms, timeout_ms)) {
        uprintf("Mass erase command failed\n");
        return;
    }
    uint32_t erase_start = AP_HAL::millis();
    uint32_t next_check_ms = 0;
    uprintf("Mass Erasing ");
    while (true) {
        if (AP_HAL::millis() > next_check_ms) {
            uprintf("\n");
            if (!jedec_dev.is_device_busy()) {
                if (next_check_ms == 0) {
                    uprintf("Sector Erase happened too fast\n");
                    return;
                }
                uprintf("Mass Erase Successful, elapsed %ld ms\n", AP_HAL::millis() - erase_start);
                return;
            } else {
                uprintf("Still busy erasing, elapsed %ld ms\n", AP_HAL::millis() - erase_start);
            }
            if ((AP_HAL::millis() - erase_start) > timeout_ms) {
                uprintf("Mass Erase Timed Out, elapsed %ld ms\n", AP_HAL::millis() - erase_start);
                return;
            }
            next_check_ms = erase_start+delay_ms;
        }
        chThdSleep(chTimeMS2I(delay_ms/100));
        uprintf("*");
    }
}

int main()
{
    init_uarts();

    while (true) {
        // Start on user input
        while (cin(0) < 0) {}
        uprintf("\n\n******************Starting Test********************\n");
        jedec_dev.init();
        test_sector_erase();
        test_page_program();
        // test_mass_erase();
        chThdSleep(chTimeMS2I(1000));
    }
}
