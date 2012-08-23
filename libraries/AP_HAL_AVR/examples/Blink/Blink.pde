
#include <AP_Common.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

const AP_HAL::HAL& hal = AP_HAL_AVR_APM1;

void loop (void) {
    hal.scheduler->delay(1000);
    hal.gpio->write(13, 1);
    hal.scheduler->delay(1000);
    hal.gpio->write(13, 0);
}

void setup (void) {
    hal.gpio->pinMode(13, GPIO_OUTPUT);
    hal.gpio->write(13, 0);
}


extern "C" {
int main (void) {
    hal.init(NULL);
    setup();
    for(;;) loop();
    return 0;
}
}
