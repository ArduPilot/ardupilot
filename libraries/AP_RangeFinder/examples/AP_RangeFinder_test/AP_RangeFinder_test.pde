/*
  AP_RangeFinder_test
  Code by DIYDrones.com
*/

// includes
#include <AP_RangeFinder.h>     // Range finder library
#include <Arduino_Mega_ISR_Registry.h>
#include <AP_PeriodicProcess.h>
#include <AP_ADC.h>				// ArduPilot Mega Analog to Digital Converter Library
#include <AP_AnalogSource.h>
#include <ModeFilter.h>			// mode filter

// declare global instances
Arduino_Mega_ISR_Registry isr_registry;
AP_TimerProcess  adc_scheduler;
AP_ADC_ADS7844	adc;
ModeFilter mode_filter;

// uncomment the appropriate line corresponding to where sonar is connected 
AP_AnalogSource_ADC	adc_source(&adc, AP_RANGEFINDER_PITOT_TYPE_ADC_CHANNEL, 0.25);    // uncomment this line to use Pitot tube
//AP_AnalogSource_Arduino adc_source(A5);   // uncomment this line to use A5 analog pin (far back-right pin on the oilpan near the CLI switch

// create the range finder object
//AP_RangeFinder_SharpGP2Y aRF(&adc_source, &mode_filter);
AP_RangeFinder_MaxsonarXL aRF(&adc_source, &mode_filter);

void setup()
{
    Serial.begin(115200);
    Serial.println("Range Finder Test v1.1");
    isr_registry.init();
    adc_scheduler.init(&isr_registry);
    adc.filter_result = true;
    adc.Init(&adc_scheduler);   // APM ADC initialization
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


