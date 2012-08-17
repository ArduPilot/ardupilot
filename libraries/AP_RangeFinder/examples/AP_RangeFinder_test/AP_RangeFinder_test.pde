/*
 *  AP_RangeFinder_test
 *  Code by DIYDrones.com
 */

// includes
#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_RangeFinder.h>     // Range finder library
#include <Arduino_Mega_ISR_Registry.h>
#include <AP_PeriodicProcess.h>
#include <AP_ADC.h>                             // ArduPilot Mega Analog to Digital Converter Library
#include <AP_AnalogSource.h>
#include <ModeFilter.h>                 // mode filter

////////////////////////////////////////////////////////////////////////////////
// Serial ports
////////////////////////////////////////////////////////////////////////////////
FastSerialPort0(Serial);        // FTDI/console

// comment out line below if using APM2 or analog pin instead of APM1's built in ADC
#define USE_ADC_ADS7844  // use APM1's built in ADC and connect sonar to pitot tube

// uncomment appropriate line corresponding to your sonar
#define SONAR_TYPE AP_RANGEFINDER_MAXSONARXL   // 0 - XL (default)
//#define SONAR_TYPE AP_RANGEFINDER_MAXSONARLV   // 1 - LV (cheaper)
//#define SONAR_TYPE AP_RANGEFINDER_MAXSONARXLL   // 2 - XLL (XL with 10m range)

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
AP_RangeFinder_MaxsonarXL aRF(&adc_source, &mode_filter);

void setup()
{
    Serial.begin(115200);
    Serial.println("Range Finder Test v1.1");
    Serial.print("Sonar Type: ");
    Serial.println(SONAR_TYPE);

    isr_registry.init();
    timer_scheduler.init(&isr_registry);

#ifdef USE_ADC_ADS7844
    adc.filter_result = true;
    adc.Init(&timer_scheduler);   // APM ADC initialization
    aRF.calculate_scaler(SONAR_TYPE,3.3);   // setup scaling for sonar
#else
    // initialise the analog port reader
    AP_AnalogSource_Arduino::init_timer(&timer_scheduler);

    aRF.calculate_scaler(SONAR_TYPE,5.0);   // setup scaling for sonar
#endif

}

void loop()
{
    Serial.print("dist:");
    Serial.print(aRF.read());
    Serial.print("\traw:");
    Serial.print(aRF.raw_value);
    Serial.println();
    delay(20);
}


