// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

//
// Simple test for the AP_AHRS interface
//

#include <AP_HAL.h>
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_InertialSensor.h>
#include <AP_ADC.h>
#include <AP_GPS.h>
#include <AP_AHRS.h>
#include <AP_Compass.h>
#include <AP_Declination.h>
#include <AP_Airspeed.h>
#include <AP_Baro.h>
#include <GCS_MAVLink.h>
#include <Filter.h>
#include <AP_Buffer.h>

#include <AP_HAL_AVR.h>

/* Only testing with APM2 for now. */
#define APM2_HARDWARE

#ifdef APM2_HARDWARE
AP_InertialSensor_MPU6000 ins;
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;
# else
AP_ADC_ADS7844 adc;
AP_InertialSensor_Oilpan ins( &adc );
const AP_HAL::HAL& hal = AP_HAL_AVR_APM1;
#endif

AP_Compass_HMC5843 compass;

GPS *g_gps;

AP_GPS_Auto g_gps_driver(hal.uart1, &g_gps);

// choose which AHRS system to use
AP_AHRS_DCM  ahrs(&ins, g_gps);
//AP_AHRS_MPU6000  ahrs(&ins, g_gps);		// only works with APM2

AP_Baro_BMP085_HIL barometer;


#define HIGH 1
#define LOW 0

#ifdef APM2_HARDWARE
 # define A_LED_PIN        27
 # define C_LED_PIN        25
 # define LED_ON           LOW
 # define LED_OFF          HIGH
 # define MAG_ORIENTATION  AP_COMPASS_APM2_SHIELD
#else
 # define A_LED_PIN        37
 # define C_LED_PIN        35
 # define LED_ON           HIGH
 # define LED_OFF          LOW
 # define MAG_ORIENTATION  AP_COMPASS_COMPONENTS_DOWN_PINS_FORWARD
#endif


static void flash_leds(bool on)
{
    hal.gpio->write(A_LED_PIN, on ? LED_OFF : LED_ON);
    hal.gpio->write(C_LED_PIN, on ? LED_ON : LED_OFF);
}

void setup(void)
{

#ifdef APM2_HARDWARE
    // we need to stop the barometer from holding the SPI bus
    hal.gpio->pinMode(40, GPIO_OUTPUT);
    hal.gpio->write(40, HIGH);
#endif

    ins.init(AP_InertialSensor::COLD_START, 
			 AP_InertialSensor::RATE_100HZ,
			 flash_leds);
    ins.init_accel(flash_leds);

    compass.set_orientation(MAG_ORIENTATION);
    ahrs.init();

    if( compass.init() ) {
        hal.console->printf("Enabling compass\n");
        ahrs.set_compass(&compass);
    } else {
        hal.console->printf("No compass detected\n");
    }
    g_gps = &g_gps_driver;
#if WITH_GPS
    g_gps->init();
#endif
}

void loop(void)
{
    static uint16_t counter;
    static uint32_t last_t, last_print, last_compass;
    uint32_t now = hal.scheduler->micros();
    float heading = 0;

    if (last_t == 0) {
        last_t = now;
        return;
    }
    last_t = now;

    if (now - last_compass > 100*1000UL &&
        compass.read()) {
        heading = compass.calculate_heading(ahrs.get_dcm_matrix());
        // read compass at 10Hz
        last_compass = now;
#if WITH_GPS
        g_gps->update();
#endif
    }

    ahrs.update();
    counter++;

    if (now - last_print >= 100000 /* 100ms : 10hz */) {
        Vector3f drift  = ahrs.get_gyro_drift();
        hal.console->printf_P(
                PSTR("r:%4.1f  p:%4.1f y:%4.1f "
                    "drift=(%5.1f %5.1f %5.1f) hdg=%.1f rate=%.1f\n"),
                        ToDeg(ahrs.roll),
                        ToDeg(ahrs.pitch),
                        ToDeg(ahrs.yaw),
                        ToDeg(drift.x),
                        ToDeg(drift.y),
                        ToDeg(drift.z),
                        compass.use_for_yaw() ? ToDeg(heading) : 0.0,
                        (1.0e6*counter)/(now-last_print));
        last_print = now;
        counter = 0;
    }
}

AP_HAL_MAIN();
