/*
 *       Example sketch to demonstrate use of DerivativeFilter library.
 */

#include <AP_HAL/AP_HAL.h>
#include <Filter/Filter.h>
#include <Filter/DerivativeFilter.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#define USE_NOISE 0

DerivativeFilter<float,11> derivative;

// setup routine
void setup(){}

static float noise(void)
{
#if USE_NOISE
    return ((random() % 100)-50) * 0.001f;
#else
    return 0;
#endif
}

//Main loop where the action takes place
void loop()
{
    hal.scheduler->delay(50);
    float t = AP_HAL::millis()*1.0e-3f;
    float s = sinf(t);
    s += noise();
    uint32_t t1 = AP_HAL::micros();
    derivative.update(s, t1);
    float output = derivative.slope() * 1.0e6f;
    hal.console->printf("%f %f %f %f\n", t, output, s, cosf(t));
}

AP_HAL_MAIN();
