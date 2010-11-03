/*
  APM_PerfMon
  Code by Randy Mackay
*/

#include "APM_PerfMon.h"        // PerfMonitor library

void setup()
{
  APM_PERFMON_REGISTER_NAME("setupA")
  Serial.begin(115200);
  Serial.println("Performance Monitor test v1.0");  
}

void loop()
{
    APM_PERFMON_REGISTER
    
    int i = 0;
    
    for( i=0; i<100; i++ )
    {
        testFunction();
    }
    
    APM_PerfMon::DisplayResults(&Serial);
 
    delay(10000); 
    
    APM_PerfMon::ClearAll();
}

void testFunction()
{
    APM_PERFMON_REGISTER
    delay(10);
    testFunction2();
    delay(10);
}

void testFunction2()
{
    APM_PERFMON_REGISTER
    delay(10);
}
