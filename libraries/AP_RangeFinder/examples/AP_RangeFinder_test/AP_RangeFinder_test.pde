/*
  AP_RangeFinder_test
  Code by DIYDrones.com
*/

#include <AP_RangeFinder.h>        // Range finder library

#define RF_PIN A5 // the far back-right pin on the oilpan (near the CLI switch)

// create the range finder object
AP_RangeFinder_SharpGP2Y aRF;
//AP_RangeFinder_MaxsonarXL aRF;

void setup()
{
  Serial.begin(115200);
  Serial.println("Range Finder Test v1.0");
  aRF.init(RF_PIN);
}

void loop()
{   
    int i = 0;
    Serial.print("dist:");
    Serial.print(aRF.read());
    Serial.print("\traw:");
    Serial.print(aRF.raw_value); 
    Serial.println();
    delay(50); 
}

