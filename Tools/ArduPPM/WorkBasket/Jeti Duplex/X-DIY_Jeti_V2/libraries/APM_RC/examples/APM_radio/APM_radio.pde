/*
  Example of APM_RC library.
  Code by Jordi Muñoz and Jose Julio. DIYDrones.com

  Print Input values and send Output to the servos
  (Works with last PPM_encoder firmware)
*/

#include <APM_RC.h> // ArduPilot Mega RC Library

void setup()
{
  APM_RC.Init();   // APM Radio initialization
  Serial.begin(57600);
  Serial.println("ArduPilot Mega RC library test");
  delay(1000);
}
void loop()
{
  if (APM_RC.GetState()==1)  // New radio frame? (we could use also if((millis()- timer) > 20)
    {
    Serial.print("CH:");
    for(int i=0;i<8;i++)
      {
      Serial.print(APM_RC.InputCh(i));  // Print channel values
      Serial.print(",");
      APM_RC.OutputCh(i,APM_RC.InputCh(i)); // Copy input to Servos
      }
    Serial.println();
    }
}