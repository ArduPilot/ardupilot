/*
* simple test of RC output interface
* Attention: If your board has safety switch,
* don't forget to push it to enable the PWM output.
*/

#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

/* change "PWM_COUNT" value accoding to the board type.
* If you are using Pixhawk,  set to 6 will enable all of the
* FMU output as PWM channels
*/
#define FMU_PWM_COUNT 6

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();
AP_BoardConfig BoardConfig;

void setup (void)
{
    hal.console->printf("Starting AP_HAL::RCOutput test\n");
    AP_Param::set_object_value(&BoardConfig, BoardConfig.var_info, "PWM_COUNT", FMU_PWM_COUNT);
    BoardConfig.init();
    for (uint8_t i = 0; i< 14; i++) {
        hal.rcout->enable_ch(i);
    }
}

static uint16_t pwm = 1500;
static int8_t delta = 1;

void loop (void)
{
    for (uint8_t i=0; i < 14; i++) {
        hal.rcout->write(i, pwm);
        pwm += delta;
        if (delta > 0 && pwm >= 2000) {
            delta = -1;
            hal.console->printf("reversing\n");
        } else if (delta < 0 && pwm <= 1000) {
            delta = 1;
            hal.console->printf("normalizing\n");
        }
    }
    hal.scheduler->delay(5);
}

AP_HAL_MAIN();
