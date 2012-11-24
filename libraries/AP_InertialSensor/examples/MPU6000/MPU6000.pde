// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

//
// Simple test for the AP_InertialSensor MPU6000 driver.
//

#include <FastSerial.h>
#include <SPI.h>
#include <Arduino_Mega_ISR_Registry.h>
#include <AP_PeriodicProcess.h>
#include <AP_ADC.h>
#include <AP_InertialSensor.h>
#include <AP_Math.h>
#include <AP_Common.h>

#define APM_HARDWARE_APM1 1
#define APM_HARDWARE_APM2 2

#define CONFIG_APM_HARDWARE APM_HARDWARE_APM2
//#define CONFIG_APM_HARDWARE APM_HARDWARE_APM1

#if CONFIG_APM_HARDWARE == APM_HARDWARE_APM2
#define SAMPLE_UNIT             1
#else
#define SAMPLE_UNIT             5               // we need 5x as many samples on the oilpan
#endif

FastSerialPort(Serial, 0);

Arduino_Mega_ISR_Registry isr_registry;
AP_TimerProcess scheduler;

#if CONFIG_APM_HARDWARE == APM_HARDWARE_APM2
AP_InertialSensor_MPU6000 ins;
#else
AP_ADC_ADS7844 adc;
AP_InertialSensor_Oilpan ins(&adc);
#endif

void setup(void)
{
    Serial.begin(115200);
    Serial.println("Doing INS startup...");

    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV16); // 1MHZ SPI rate

    isr_registry.init();
    scheduler.init(&isr_registry);

    // we need to stop the barometer from holding the SPI bus
    pinMode(40, OUTPUT);
    digitalWrite(40, HIGH);

#if CONFIG_APM_HARDWARE == APM_HARDWARE_APM1
    adc.Init(&scheduler);           // APM ADC library initialization
#endif
    ins.init(AP_InertialSensor::COLD_START, delay, NULL, &scheduler);

    // display initial values
    display_offsets_and_scaling();
}

void loop(void)
{
    int16_t user_input;

    Serial.println();
    Serial.println("Menu: ");
    Serial.println("    c) calibrate accelerometers");
    Serial.println("    d) display offsets and scaling");
    Serial.println("    l) level (capture offsets from level)");
    Serial.println("    t) test");

    // wait for user input
    while( !Serial.available() ) {
        delay(20);
    }

    // read in user input
    while( Serial.available() ) {
        user_input = Serial.read();

        if( user_input == 'c' || user_input == 'C' ) {
            run_calibration();
            display_offsets_and_scaling();
        }

        if( user_input == 'd' || user_input == 'D' ) {
            display_offsets_and_scaling();
        }

        if( user_input == 'l' || user_input == 'L' ) {
            run_level();
            display_offsets_and_scaling();
        }

        if( user_input == 't' || user_input == 'T' ) {
            run_test();
        }
    }
}

static void setup_printf_P(const prog_char_t *fmt, ...)
{
    va_list arg_list;
    va_start(arg_list, fmt);
    Serial.vprintf_P(fmt, arg_list);
    va_end(arg_list);
}

static void setup_wait_key(void)
{
    // wait for user input
    while (!Serial.available()) {
        delay(20);
    }
    // clear input buffer
    while( Serial.available() ) {
        Serial.read();
    }
}

void run_calibration()
{
    // clear off any other characters (like line feeds,etc)
    while( Serial.available() ) {
        Serial.read();
    }

    ins.calibrate_accel(delay, NULL, setup_printf_P, setup_wait_key);
}

void display_offsets_and_scaling()
{
    Vector3f accel_offsets = ins.get_accel_offsets();
    Vector3f accel_scale = ins.get_accel_scale();
    Vector3f gyro_offsets = ins.get_gyro_offsets();

    // display results
    Serial.printf_P(PSTR("\nAccel Offsets X:%10.8f \t Y:%10.8f \t Z:%10.8f\n"),
                    accel_offsets.x,
                    accel_offsets.y,
                    accel_offsets.z);
    Serial.printf_P(PSTR("Accel Scale X:%10.8f \t Y:%10.8f \t Z:%10.8f\n"),
                    accel_scale.x,
                    accel_scale.y,
                    accel_scale.z);
    Serial.printf_P(PSTR("Gyro Offsets X:%10.8f \t Y:%10.8f \t Z:%10.8f\n"),
                    gyro_offsets.x,
                    gyro_offsets.y,
                    gyro_offsets.z);
}

void run_level()
{
    // clear off any input in the buffer
    while( Serial.available() ) {
        Serial.read();
    }

    // display message to user
    Serial.print("Place APM on a level surface and press any key..\n");

    // wait for user input
    while( !Serial.available() ) {
        delay(20);
    }
    while( Serial.available() ) {
        Serial.read();
    }

    // run accel level
    ins.init_accel(delay, NULL);

    // display results
    display_offsets_and_scaling();
}

void run_test()
{
    Vector3f accel;
    Vector3f gyro;
    float temperature;
    float length;

    // flush any user input
    while( Serial.available() ) {
        Serial.read();
    }

    // clear out any existing samples from ins
    ins.update();

    // loop as long as user does not press a key
    while( !Serial.available() ) {

        // wait until we have 8 samples
        while( ins.num_samples_available() < 8 * SAMPLE_UNIT ) {
            delay(1);
        }

        // read samples from ins
        ins.update();
        accel = ins.get_accel();
        gyro = ins.get_gyro();
        temperature = ins.temperature();

        length = sqrt(accel.x*accel.x + accel.y*accel.y + accel.z*accel.z);

        // display results
        Serial.printf_P(PSTR("Accel X:%4.2f \t Y:%4.2f \t Z:%4.2f \t len:%4.2f \t Gyro X:%4.2f \t Y:%4.2f \t Z:%4.2f \t Temp:%4.2f\n"), 
            accel.x, accel.y, accel.z, length, gyro.x, gyro.y, gyro.z, temperature);
    }

    // clear user input
    while( Serial.available() ) {
        Serial.read();
    }
}
