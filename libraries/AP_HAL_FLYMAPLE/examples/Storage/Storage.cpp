
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_FLYMAPLE/AP_HAL_FLYMAPLE.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

uint8_t fibs[] = { 1, 1, 2, 3, 5, 8, 13, 21, 34, 55, 89, 144, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55, 89, 144, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55, 89, 144, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55, 89, 144, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55, 89, 144, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55, 89, 144, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55, 89, 144, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55, 89, 144, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55, 89, 144, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55, 89, 144, 0 };

void test_erase() {
    hal.console->printf("erasing... ");
    for(int i = 0; i < 100; i++) {
        hal.storage->write_byte(i, 0);
    }
    hal.console->printf(" done.\r\n");
}

void test_write() {
    hal.console->printf("writing... ");
    hal.storage->write_block(0, fibs, sizeof(fibs));
    hal.console->printf(" done.\r\n");
}

void test_readback() {
    hal.console->printf("reading back...\r\n");
    uint8_t readback[sizeof(fibs)];
    bool success = true;
    hal.storage->read_block(readback, 0, sizeof(fibs));
    for (int i = 0; i < sizeof(fibs); i++) {
        if (readback[i] != fibs[i]) {
            success = false;
            hal.console->printf("At index %d expected %d got %d\r\n",
                    i, (int) fibs[i], (int) readback[i]); 
        }   
    }
    if (success) {
        hal.console->printf("all bytes read successfully\r\n");
    }
    hal.console->printf("done reading back.\r\n");
}

void setup (void) {
    hal.scheduler->delay(5000);
    hal.console->printf("Starting AP_HAL_FLYMAPLE::Storage test\r\n");

    hal.console->printf("test %d\n", i);
    test_readback(); // Test what was left from the last run, possibly after power off
    test_erase();
    test_write();
    test_readback();
}

void loop (void) { }

AP_HAL_MAIN();
