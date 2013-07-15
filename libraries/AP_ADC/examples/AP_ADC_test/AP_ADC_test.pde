
/*
 *  Example of APM_ADC library.
 *  Code by Jordi Muñoz and Jose Julio. DIYDrones.com
 */

#include <math.h>

#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_Progmem.h>

#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

#include <AP_ADC.h>

uint32_t timer;

#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
/* Only build this sketch for APM1 */
const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

AP_ADC_ADS7844 adc;

void setup()
{
    hal.console->println("ArduPilot Mega ADC library test");
    hal.scheduler->delay(1000);

    adc.Init();       // APM ADC initialization

    hal.scheduler->delay(1000);
    timer = hal.scheduler->millis();
}

static const uint8_t channel_map[6] = { 1, 2, 0, 4, 5, 6};
float v;
uint32_t last_usec = 0;

static void show_timing()
{
    uint32_t mint = (uint32_t)-1, maxt = 0, totalt=0;
    uint32_t start_time = hal.scheduler->millis();
    float result[6];
    uint32_t count = 0;

    hal.console->println("Starting timing test");

    adc.Ch6(channel_map, result);

    do {
        uint32_t deltat = adc.Ch6(channel_map, result);
        if (deltat > maxt) maxt = deltat;
        if (deltat < mint) mint = deltat;
        totalt += deltat;
        count++;
    } while ((hal.scheduler->millis() - start_time) < 5000);

    hal.console->printf("timing: mint=%lu maxt=%lu avg=%lu\n", mint, maxt, totalt/count);
}

static void show_data()
{
    float result[6];
    uint32_t deltat = 0;
    uint16_t ch3;
    float min[6], max[6];
    uint8_t i;

    for (i=0; i<6; i++) {
        /* clearly out of bounds values: */
        min[i] = 99999999.0f;
        max[i] = -88888888.0f;
    } 


    do {
        ch3 = adc.Ch(3);
        deltat += adc.Ch6(channel_map, result);
        for (i=0; i<6; i++) {
            if (result[i] < min[i]) min[i] = result[i];
            if (result[i] > max[i]) max[i] = result[i];
            if (fabsf(result[i]) > 0x8000) {
                hal.console->printf("result[%u]=%f\n", (unsigned)i, result[i]);
            }
        }
    } while ((hal.scheduler->millis() - timer) < 200);

    timer = hal.scheduler->millis();
    hal.console->printf("g=(%f,%f,%f) a=(%f,%f,%f) +/-(%.0f,%.0f,%.0f,%.0f,%.0f,%.0f) gt=%u dt=%u\n",
                  result[0], result[1], result[2],
                  result[3], result[4], result[5],
                  (max[0]-min[0]), (max[1]-min[1]), (max[2]-min[2]),
                  (max[3]-min[3]), (max[4]-min[4]), (max[5]-min[5]),
                  ch3, (unsigned)(deltat/1000));
}


void loop()
{
    if (hal.scheduler->millis() < 5000) {
        show_timing();
    } else {
        show_data();
    }
}

#else // Non-APM1:
#warning AP_ADC_test built as stub for APM2
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;
void setup () {}
void loop () {}
#endif  // CONFIG_HAL_BOARD

AP_HAL_MAIN();
