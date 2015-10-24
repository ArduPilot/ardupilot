/*
 *       Example sketch to demonstrate use of DerivativeFilter library.
 */

#include <AP_HAL/AP_HAL.h>
#include <Filter/Filter.h>
#include <Filter/DerivativeFilter.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

DerivativeFilter<float,11> derivative;

// setup routine
void setup(){}

static float noise(void)
{
    return ((random() % 100)-50) * 0.001f;
}

//Main loop where the action takes place
void loop()
{
    hal.scheduler->delay(50);
    float t = hal.scheduler->millis()*1.0e-3f;
    float s = sinf(t);
    //s += noise();
    uint32_t t1 = hal.scheduler->micros();
    derivative.update(s, t1);
    float output = derivative.slope() * 1.0e6f;
    hal.console->printf("%f %f %f %f\n", t, output, s, cosf(t));
}

AP_HAL_MAIN();
