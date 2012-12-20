/*
  AP_PerfMon
  Code by Randy Mackay
*/

#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_PerfMon.h>        // PerfMonitor library

FastSerialPort0(Serial);

void setup()
{
  AP_PERFMON_REGISTER_NAME("setupA")
  Serial.begin(115200);

  Serial.println("Performance Monitor test v1.1");
  // let tx buffer clear & display available space
  delay(100);
  Serial.printf_P(PSTR("Tx buf:%d available:%d\n"),Serial.txspace(),Serial.available());
  delay(100);
  Serial.set_blocking_writes(false);
}

void loop()
{
    AP_PERFMON_REGISTER

    int16_t i = 0;

    for( i=0; i<10; i++ ) {
        testFn();
    }

    //AP_PerfMon::DisplayAndClear(5);
    AP_PerfMon::DisplayResults();
    AP_PerfMon::ClearAll();

    delay(10000);
}

void testFn()
{
    AP_PERFMON_REGISTER
    //delay(10);
    //testFn2();
    //delay(10);
    Serial.printf_P(PSTR("1234567890"));
}

void testFn2()
{
    AP_PERFMON_REGISTER
    delay(10);
}
