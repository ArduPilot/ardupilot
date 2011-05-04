/*
  AP_RangeFinder_test
  Code by DIYDrones.com
*/

#include <AP_RangeFinder.h>     // Range finder library
#include <AP_ADC.h>		// ArduPilot Mega Analog to Digital Converter Library

#define RF_PIN AP_RANGEFINDER_PITOT_TUBE // the pitot tube on the front of the oilpan
//#define RF_PIN A5 // A5 is the far back-right pin on the oilpan (near the CLI switch)

// declare global instances for reading pitot tube
AP_ADC_ADS7844	adc;

// create the range finder object
//AP_RangeFinder_SharpGP2Y aRF;
AP_RangeFinder_MaxsonarXL aRF;
//AP_RangeFinder_MaxsonarLV aRF;

void setup()
{
  Serial.begin(38400);
  Serial.println("Range Finder Test v1.0");
  adc.Init();            // APM ADC library initialization
  aRF.init(RF_PIN, &adc);
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


