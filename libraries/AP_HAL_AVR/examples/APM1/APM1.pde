
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

const AP_HAL::HAL& hal1 = AP_HAL_AVR_APM1;
const AP_HAL::HAL& hal2 = AP_HAL_AVR_APM2;

void loop (void) {

}

void setup (void) {
    hal1.uart0->init(9600);
    hal2.uart0->init(9600);
}


