/*
 *  AP_RangeFinder_test
 *  Code by DIYDrones.com
 */

// includes
#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Math.h>
#include <Filter.h>
#include <I2C.h>                // Arduino I2C lib
#include <AP_RangeFinder.h>     // Range finder library
#include <Arduino_Mega_ISR_Registry.h>
#include <AP_PeriodicProcess.h>
#include <AP_ADC.h>                             // ArduPilot Mega Analog to Digital Converter Library
#include <AP_AnalogSource.h>
#include <ModeFilter.h>                 // mode filter
#include <AP_Buffer.h>

#define APM_HARDWARE_APM1   1
#define APM_HARDWARE_APM2   2

// uncomment appropriate line for your APM hardware type
//#define CONFIG_APM_HARDWARE APM_HARDWARE_APM1
#define CONFIG_APM_HARDWARE APM_HARDWARE_APM2   // also applies to APM2.5

// uncomment appropriate line corresponding to your sonar
#define SONAR_TYPE AP_RANGEFINDER_MAXSONARXL      // 0 - XL (default)
//#define SONAR_TYPE AP_RANGEFINDER_MAXSONARLV      // 1 - LV (cheaper)
//#define SONAR_TYPE AP_RANGEFINDER_MAXSONARXLL     // 2 - XLL (XL with 10m range)
//#define SONAR_TYPE AP_RANGEFINDER_MAXSONARHRLV    // 3 - HRLV-MaxSonar-EZ0 (5m range)
//#define SONAR_TYPE AP_RANGEFINDER_MAXSONARI2CXL   // 4 - XLI2C (XL with I2C interface and 7m range)

// For APM1 we use built in ADC for sonar reads from an analog pin
#if CONFIG_APM_HARDWARE == APM_HARDWARE_APM1 && SONAR_TYPE <= AP_RANGEFINDER_MAXSONARHRLV
# define USE_ADC_ADS7844  // use APM1's built in ADC and connect sonar to pitot tube
#endif

////////////////////////////////////////////////////////////////////////////////
// Serial ports
////////////////////////////////////////////////////////////////////////////////
FastSerialPort0(Serial);        // FTDI/console

// define Pitot tube's ADC Channel
#define AP_RANGEFINDER_PITOT_TYPE_ADC_CHANNEL 7

// declare global instances
Arduino_Mega_ISR_Registry isr_registry;
ModeFilterInt16_Size5 mode_filter(2);
AP_TimerProcess timer_scheduler;

#ifdef USE_ADC_ADS7844
AP_ADC_ADS7844 adc;
AP_AnalogSource_ADC adc_source(&adc, AP_RANGEFINDER_PITOT_TYPE_ADC_CHANNEL, 0.25);        // use Pitot tube
#else
AP_AnalogSource_Arduino adc_source(A0);       // use AN0 analog pin for APM2 on left
#endif

// create the range finder object
//AP_RangeFinder_SharpGP2Y aRF(&adc_source, &mode_filter);

// declare maxbotix sensors using analog pin
#if SONAR_TYPE >= AP_RANGEFINDER_MAXSONARXL && SONAR_TYPE <= AP_RANGEFINDER_MAXSONARHRLV
AP_RangeFinder_MaxsonarXL aRF(&adc_source, &mode_filter);
#endif

// declare I2C maxbotix sensor
#if SONAR_TYPE == AP_RANGEFINDER_MAXSONARI2CXL
AP_RangeFinder_MaxsonarI2CXL aRF(&mode_filter);
#endif

void setup()
{
    Serial.begin(115200);
    Serial.println("Range Finder Test v1.1");
    Serial.print("Sonar Type: ");
    Serial.println(SONAR_TYPE);

    isr_registry.init();
    timer_scheduler.init(&isr_registry);

    // initialise communication method (analog read or i2c)
#if SONAR_TYPE == AP_RANGEFINDER_MAXSONARI2CXL
    I2c.begin();
    I2c.timeOut(5);
    I2c.setSpeed(true); // initially set a fast I2c speed, and drop it on first failures
#else
# ifdef USE_ADC_ADS7844
    adc.Init(&timer_scheduler);   // APM ADC initialization
    aRF.calculate_scaler(SONAR_TYPE,3.3);   // setup scaling for sonar
# else
    // initialise the analog port reader
    AP_AnalogSource_Arduino::init_timer(&timer_scheduler);
    aRF.calculate_scaler(SONAR_TYPE,5.0);   // setup scaling for sonar
# endif
#endif
}

void loop()
{
    Serial.print("dist:");
    Serial.print(aRF.read());
    Serial.print("\traw:");
    Serial.print(aRF.raw_value);
    Serial.println();

#if SONAR_TYPE == AP_RANGEFINDER_MAXSONARI2CXL
    if( !aRF.healthy ) {
        Serial.println("not healthy!");
    }
    aRF.take_reading();
#endif
    delay(100);
}
