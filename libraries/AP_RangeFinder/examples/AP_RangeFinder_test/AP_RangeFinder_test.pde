/*
  AP_RangeFinder_test
  Code by DIYDrones.com
*/

#include <AP_RangeFinder.h>     // Range finder library
#include <AP_ADC.h>				// ArduPilot Mega Analog to Digital Converter Library
#include <ModeFilter.h>			// mode filter

#define RF_PIN AP_RANGEFINDER_PITOT_TUBE // the pitot tube on the front of the oilpan
//#define RF_PIN A5 // A5 is the far back-right pin on the oilpan (near the CLI switch)

// declare global instances for reading pitot tube
AP_ADC_ADS7844	adc;
ModeFilter mode_filter;

// create the range finder object
//AP_RangeFinder_SharpGP2Y aRF(&adc, &mode_filter);
AP_RangeFinder_MaxsonarXL aRF(&adc, &mode_filter);

void setup()
{
  Serial.begin(115200);
  Serial.println("Range Finder Test v1.0");
  adc.Init();            // APM ADC library initialization
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


