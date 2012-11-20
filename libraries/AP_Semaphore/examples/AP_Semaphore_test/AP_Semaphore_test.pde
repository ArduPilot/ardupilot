
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_HAL.h>
#include <AP_Semaphore.h>

#include <AP_HAL_AVR.h>
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;

static AP_Semaphore my_semaphore;
static AP_Semaphore AP_Semaphore_spi3;

int16_t dummy1, dummy2, dummy3; // used to provide points to semaphore library

// call back for dummy1
void callback_1() {
    hal.console->println("dummy2 called back!");
}

// call back for dummy3
void callback_3() {
    hal.console->println("dummy3 called back!");
}

void setup(void)
{
    // print welcome message
    hal.console->println("AP_Semaphore_test ver 1.0");
}

void print_test_result(bool expected, bool got) {
    if( expected == got ) {
        hal.console->print_P(PSTR("success: got "));
        if (got) {
            hal.console->println_P(PSTR("true"));
        } else {
            hal.console->println_P(PSTR("false"));
        }
    } else {
        hal.console->printf_P(PSTR("***failed************************: expected %d got %d\r\n"),
                (int)expected, (int) got);
    }
}

void loop(void)
{
    bool ret;

    // quick test of spi semaphore
    ret = AP_Semaphore_spi3.get(&dummy3);
    hal.console->print("dummy3 gets SPI semaphore: ");
    print_test_result(true, ret);

    // dummy1 gets semaphore    
    hal.console->print("dummy1 gets semaphore: ");
    ret = my_semaphore.get(&dummy1);
    print_test_result(true, ret);

    // dummy2 tries to get semaphore (fails)
    hal.console->print("dummy2 gets semaphore: ");
    ret = my_semaphore.get(&dummy2);
    print_test_result(false, ret);

    // dummy2 tries to release semaphore (fails)
    hal.console->print("dummy2 releases semaphore (that it doesn't have): ");
    ret = my_semaphore.release(&dummy2);
    print_test_result(false, ret);

    // dummy1 releases semaphore
    hal.console->print("dummy1 releases semaphore: ");
    ret = my_semaphore.release(&dummy1);
    print_test_result(true, ret);

    // dummy2 tries to get semaphore (succeeds)
    hal.console->print("dummy2 gets semaphore: ");
    ret = my_semaphore.get(&dummy2);
    print_test_result(true, ret);

    // dummy1 tries to get semphore (fails)
    hal.console->print("dummy1 gets semaphore: ");
    ret = my_semaphore.get(&dummy1);
    print_test_result(false, ret);
    
    // dummy1 asks for call back (succeeds)
    hal.console->print("dummy1 asks for call back on release: ");
    ret = my_semaphore.call_on_release(&dummy1, callback_1);
    print_test_result(true, ret);
    
    // dummy3 asks for call back (fails)
    hal.console->print("dummy3 asks for call back on release: ");
    ret = my_semaphore.call_on_release(&dummy3, callback_3);
    print_test_result(false, ret);

    // dummy2 releases semaphore
    // dummy1's call back should be called
    hal.console->print("dummy2 releases semaphore: ");
    ret = my_semaphore.release(&dummy2);
    print_test_result(true, ret);

    hal.console->println("--------------------");

    // nobody has semaphore

    // delay
    hal.scheduler->delay(10000);
}

extern "C" {
int main (void) {
    hal.init(NULL);
    setup();
    for(;;) loop();
    return 0;
}
}
