/*
  Example of APM_ADC library.
  Code by Jordi Muñoz and Jose Julio. DIYDrones.com
*/

#include <APM_ADC.h> // ArduPilot Mega ADC Library

unsigned long timer;

void setup()
{  
  APM_ADC.Init();   // APM ADC initialization
  Serial.begin(57600);
  Serial.println("ArduPilot Mega ADC library test");
  delay(1000);
  timer = millis();
}

void loop()
{
  int ch;
  
  if((millis()- timer) > 20)
    {
    timer = millis();
    for (ch=0;ch<7;ch++)
      {
      Serial.print(APM_ADC.Ch(ch));
      Serial.print(",");
      }
    Serial.println();
    }
}