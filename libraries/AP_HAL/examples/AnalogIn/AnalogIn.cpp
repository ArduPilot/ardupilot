#include <AP_HAL/AP_HAL.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

AP_HAL::AnalogSource* ch;

void setup(void) {
    hal.console->printf("Starting AP_HAL::AnalogIn test\r\n");
    ch = hal.analogin->channel(0);
}

static int8_t pin;

void loop(void)
{
    float v  = ch->voltage_average();
    if (pin == 0) {
        hal.console->printf("\n");
    }
    hal.console->printf("[%u %.3f] ",
              (unsigned)pin, (double)v);
    pin = (pin+1) % 16;
    ch->set_pin(pin);
    hal.scheduler->delay(100);
}

AP_HAL_MAIN();
