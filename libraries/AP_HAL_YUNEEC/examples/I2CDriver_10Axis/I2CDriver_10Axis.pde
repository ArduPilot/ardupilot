/*******************************************
*   Sample sketch that configures an HMC5883L 3 axis
*   magnetometer to continuous mode and reads back
*   the three axis of data.
*******************************************/
#include <stdarg.h>

#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_Progmem.h>

#include <AP_HAL.h>
#include <AP_HAL_YUNEEC.h>
#include <AP_Baro.h>
#include <AP_ADC.h>
#include <AP_InertialSensor.h>
#include <AP_Notify.h>
#include <AP_GPS.h>
#include <DataFlash.h>
#include <GCS_MAVLink.h>
#include <AP_Mission.h>
#include <AP_AHRS.h>
#include <AP_Airspeed.h>
#include <AP_Vehicle.h>
#include <AP_ADC_AnalogSource.h>
#include <AP_Compass.h>
#include <AP_Declination.h>
#include <Filter.h>             // Filter library
#include <utility/pinmap_typedef.h>



const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

AP_HAL::DigitalSource *led;
static AP_Baro_MS5611 baro(&AP_Baro_MS5611::i2c);
static AP_InertialSensor_MPU6050 ins;
static AP_Compass_HMC5843 compass;
static uint32_t timer;

void setup(void)
{

    hal.gpio->pinMode(PC13, HAL_GPIO_OUTPUT);
    hal.gpio->write(PC13, 0);

    hal.console->println("AP_InertialSensor startup...");

    ins.init(AP_InertialSensor::COLD_START,
			 AP_InertialSensor::RATE_400HZ);

    // display initial values
    display_offsets_and_scaling();

    hal.scheduler->delay(1000);

    hal.console->println("YUNEEC MS5611 Barometer library test");
    baro.init();
    baro.calibrate();
    timer = hal.scheduler->micros();

    if( compass.init() ) {
        hal.console->printf("Enabling compass\n");
    } else {
        hal.console->printf("No compass detected\n");
    }
}

void display_offsets_and_scaling()
{
    Vector3f accel_offsets = ins.get_accel_offsets();
    Vector3f accel_scale = ins.get_accel_scale();
    Vector3f gyro_offsets = ins.get_gyro_offsets();

    // display results
    hal.console->printf_P(
            PSTR("\nAccel Offsets X:%10.8f \t Y:%10.8f \t Z:%10.8f\n"),
                    accel_offsets.x,
                    accel_offsets.y,
                    accel_offsets.z);
    hal.console->printf_P(
            PSTR("Accel Scale X:%10.8f \t Y:%10.8f \t Z:%10.8f\n"),
                    accel_scale.x,
                    accel_scale.y,
                    accel_scale.z);
    hal.console->printf_P(
            PSTR("Gyro Offsets X:%10.8f \t Y:%10.8f \t Z:%10.8f\n"),
                    gyro_offsets.x,
                    gyro_offsets.y,
                    gyro_offsets.z);
}

void run_test()
{
    Vector3f accel;
    Vector3f gyro;
    float length;

    // clear out any existing samples from ins
//    ins.update();

	// wait until we have a sample
//	ins.wait_for_sample(1000);

	// read samples from ins
	ins.update();
	accel = ins.get_accel();
	gyro = ins.get_gyro();

	length = accel.length();

	// display results
	hal.console->printf_P(PSTR("Accel X:%4.2f \t Y:%4.2f \t Z:%4.2f \t len:%4.2f \n Gyro X:%4.2f \t Y:%4.2f \t Z:%4.2f\n"),
						  accel.x, accel.y, accel.z, length, gyro.x, gyro.y, gyro.z);

}

void loop()
{
    static uint32_t last_t, last_compass;
    uint32_t now = hal.scheduler->micros();
    float heading = 0;

    if (last_t == 0) {
        last_t = now;
        return;
    }
    last_t = now;

    if (now - last_compass > 100*1000UL &&
        compass.read()) {
        // read compass at 10Hz
        last_compass = now;
    }

    if((hal.scheduler->micros() - timer) > 100000UL) {
        timer = hal.scheduler->micros();
        baro.read();
        uint32_t read_time = hal.scheduler->micros() - timer;
        if (!baro.healthy) {
            hal.console->println("not healthy");
            return;
        }
        hal.console->print("Lock up count: ");
        hal.console->print(hal.i2c->lockup_count());
        hal.console->print(" Pressure:");
        hal.console->print(baro.get_pressure());
        hal.console->print(" Temperature:");
        hal.console->print(baro.get_temperature());
        hal.console->print(" Altitude:");
        hal.console->print(baro.get_altitude());
        hal.console->printf(" climb=%.2f t=%u samples=%u",
                      baro.get_climb_rate(),
                      (unsigned)read_time,
                      (unsigned)baro.get_pressure_samples());
        hal.console->println();

        run_test();
    }
}

AP_HAL_MAIN();
