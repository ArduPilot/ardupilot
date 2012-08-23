
#include <AP_Common.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

const AP_HAL::HAL& hal = AP_HAL_AVR_APM1;
const AP_HAL::HAL& hal2 = AP_HAL_AVR_APM2;

void loop (void) {

}

void setup (void) {
    hal.uart0->begin(9600);
    hal2.uart0->begin(9600);
    hal.uart1->begin(9600);
    hal2.uart1->begin(9600);
    hal.uart2->begin(9600);
    hal2.uart2->begin(9600);
    hal.uart3->begin(9600);
    hal2.uart3->begin(9600);
}


extern "C" {
int main (void) {
    hal.init(NULL);
    setup();
    for(;;) loop();
    return 0;
}
}
