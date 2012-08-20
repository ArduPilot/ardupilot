/*
 *       Example sketch to demonstrate use of DerivativeFilter library.
 */

#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <Filter.h>
#include <DerivativeFilter.h>
#include <AP_Buffer.h>

#ifdef DESKTOP_BUILD
// all of this is needed to build with SITL
 #include <DataFlash.h>
 #include <APM_RC.h>
 #include <GCS_MAVLink.h>
 #include <Arduino_Mega_ISR_Registry.h>
 #include <AP_PeriodicProcess.h>
 #include <AP_ADC.h>
 #include <AP_Baro.h>
 #include <AP_Compass.h>
 #include <AP_GPS.h>
 #include <Filter.h>
 #include <SITL.h>
 #include <I2C.h>
 #include <SPI.h>
 #include <AP_Declination.h>
 #include <AP_Semaphore.h>
Arduino_Mega_ISR_Registry isr_registry;
AP_Baro_BMP085_HIL barometer;
AP_Compass_HIL compass;
SITL sitl;
#endif

FastSerialPort0(Serial);        // FTDI/console

DerivativeFilter<float,11> derivative;

// setup routine
void setup()
{
    // Open up a serial connection
    Serial.begin(115200);
}

static float noise(void)
{
    return ((random() % 100)-50) * 0.001;
}

//Main loop where the action takes place
void loop()
{
    delay(50);
    float t = millis()*1.0e-3;
    float s = sin(t);
    //s += noise();
    uint32_t t1 = micros();
    derivative.update(s, t1);
    float output = derivative.slope() * 1.0e6;
    uint32_t t2 = micros();
    Serial.printf("%f %f %f %f\n", t, output, s, cos(t));
}
