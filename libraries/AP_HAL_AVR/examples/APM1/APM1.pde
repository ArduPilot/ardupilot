
#include <AP_Common.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

const AP_HAL::HAL& hal = AP_HAL_AVR_APM1;

void loop (void) {
    hal.uart0->println(".");
    hal.gpio->write(3, 0);
    hal.scheduler->delay(1000);
    hal.gpio->write(3, 1);
}

void setup (void) {
    hal.gpio->pinMode(1, GPIO_OUTPUT);
    hal.gpio->pinMode(2, GPIO_OUTPUT);
    hal.gpio->pinMode(3, GPIO_OUTPUT);
    hal.gpio->pinMode(13, GPIO_OUTPUT);

    hal.gpio->write(1, 1);
    hal.gpio->write(2, 1);
    hal.gpio->write(3, 1);
    hal.gpio->write(13, 1);
    
    hal.scheduler->delay(1000);

    hal.gpio->write(1, 0);
    hal.gpio->write(2, 0);
    hal.gpio->write(13, 0);
    
    hal.scheduler->delay(1000);

    hal.gpio->write(13, 1);

    hal.uart0->begin(115200);
    hal.gpio->write(1, 0);
    hal.uart0->println("Hello World");
    hal.gpio->write(2, 0);
}


extern "C" {
int main (void) {
    hal.init(NULL);
    setup();
    for(;;) loop();
    return 0;
}
}
