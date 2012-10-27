
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_HAL.h>
#include <AP_HAL_PX4.h>

const AP_HAL::HAL& hal = AP_HAL_PX4_Instance;

void loop (void) {
    hal.console->println(".");
}

void setup (void) {
    hal.console->println("Hello World");
}


extern "C" {
int main (void) {
    hal.init(NULL);
    setup();
    for(;;) loop();
    return 0;
}
}
