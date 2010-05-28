/*
  Example of APM_Compass library (HMC5843 sensor).
  Code by Jordi Muñoz and Jose Julio. DIYDrones.com
*/

#include <Wire.h>
#include <APM_Compass.h> // Compass Library

#define ToDeg(x) (x*57.2957795131)  // *180/pi

unsigned long timer;

void setup()
{  
  APM_Compass.Init();   // Initialization
  Serial.begin(57600);
  Serial.println("Compass library test (HMC5843)");
  delay(1000);
  timer = millis();
}

void loop()
{
  float tmp;
  
  if((millis()- timer) > 100)
    {
    timer = millis();
    APM_Compass.Read();
    APM_Compass.Calculate(0,0);  // roll = 0, pitch = 0 for this example
    Serial.print("Heading:");
    Serial.print(ToDeg(APM_Compass.Heading));
    Serial.print("  (");
    Serial.print(APM_Compass.Mag_X);
    Serial.print(",");
    Serial.print(APM_Compass.Mag_Y);
    Serial.print(",");
    Serial.print(APM_Compass.Mag_Z);
    Serial.println();
    }
}