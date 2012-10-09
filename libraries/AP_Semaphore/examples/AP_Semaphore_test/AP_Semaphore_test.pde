
#include <FastSerial.h>
#include <AP_Math.h>
#include <AP_Common.h>
#include <AP_Semaphore.h>

FastSerialPort0(Serial);

AP_Semaphore my_semaphore;

int16_t dummy1, dummy2, dummy3; // used to provide points to semaphore library

// call back for dummy1
void callback_1() {
    Serial.println("dummy2 called back!");
}

// call back for dummy3
void callback_3() {
    Serial.println("dummy3 called back!");
}

void setup(void)
{
    // initialise serial port
    Serial.begin(115200);

    // print welcome message
    Serial.println("AP_Semaphore_test ver 1.0");
}

void print_true_false(bool true_false) {
    if( true_false ) {
        Serial.println("success");
    } else {
        Serial.println("failed");
    }
}

void loop(void)
{
    bool ret;

    // quick test of spi semaphore
    ret = AP_Semaphore_spi3.get(&dummy3);
    Serial.print("dummy3 gets SPI semaphore: ");
    print_true_false(ret);

    // dummy1 gets semaphore    
    Serial.print("dummy1 gets semaphore: ");
    ret = my_semaphore.get(&dummy1);
    print_true_false(ret);

    // dummy2 tries to get semaphore (fails)
    Serial.print("dummy2 gets semaphore: ");
    ret = my_semaphore.get(&dummy2);
    print_true_false(ret);

    // dummy2 tries to release semaphore (fails)
    Serial.print("dummy2 releases semaphore (that it doesn't have): ");
    ret = my_semaphore.release(&dummy2);
    print_true_false(ret);

    // dummy1 releases semaphore
    Serial.print("dummy1 releases semaphore: ");
    ret = my_semaphore.release(&dummy1);
    print_true_false(ret);

    // dummy2 tries to get semaphore (succeeds)
    Serial.print("dummy2 gets semaphore: ");
    ret = my_semaphore.get(&dummy2);
    print_true_false(ret);

    // dummy1 tries to get semphore (fails)
    Serial.print("dummy1 gets semaphore: ");
    ret = my_semaphore.get(&dummy1);
    print_true_false(ret);
    
    // dummy1 asks for call back (succeeds)
    Serial.print("dummy1 asks for call back on release: ");
    ret = my_semaphore.call_on_release(&dummy1, callback_1);
    print_true_false(ret);
    
    // dummy3 asks for call back (fails)
    Serial.print("dummy3 asks for call back on release: ");
    ret = my_semaphore.call_on_release(&dummy3, callback_3);
    print_true_false(ret);

    // dummy2 releases semaphore
    // dummy1's call back should be called
    Serial.print("dummy2 releases semaphore: ");
    ret = my_semaphore.release(&dummy2);
    print_true_false(ret);

    Serial.println("--------------------");

    // nobody has semaphore

    // delay
    delay(10000);
}

