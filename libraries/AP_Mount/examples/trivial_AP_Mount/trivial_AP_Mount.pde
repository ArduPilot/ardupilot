
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <AP_HAL.h>

#include <AP_GPS.h>
#include <AP_AHRS.h>
#include <AP_InertialSensor.h>
#include <AP_Compass.h>
#include <AP_Declination.h>
#include <AP_Airspeed.h>
#include <AP_ADC.h>
#include <AP_Baro.h>
#include <AP_Buffer.h>
#include <Filter.h>
#include <GCS_MAVLink.h>
#include <RC_Channel.h>

#include <AP_Mount.h>

#include <AP_HAL_AVR.h>
const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

void setup () {
    hal.console->println_P(PSTR("Unit test for AP_Mount. This sketch"
                "has no functionality, it only tests build."));
}
void loop () {}

AP_HAL_MAIN();
