/*
 *       Example of AP Lead_Filter library.
 *       Code by Jason Short. 2010
 *       DIYDrones.com
 *
 */

#include <FastSerial.h>
#include <AP_Common.h>          // ArduPilot Mega Common Library
#include <Arduino_Mega_ISR_Registry.h>
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <AP_LeadFilter.h>      // GPS Lead filter

// define APM1 or APM2
#define APM_HARDWARE_APM1       1
#define APM_HARDWARE_APM2       2

// set your hardware type here
#define CONFIG_APM_HARDWARE APM_HARDWARE_APM2

Arduino_Mega_ISR_Registry isr_registry;

////////////////////////////////////////////////////////////////////////////////
// Serial ports
////////////////////////////////////////////////////////////////////////////////
//
// Note that FastSerial port buffers are allocated at ::begin time,
// so there is not much of a penalty to defining ports that we don't
// use.
//
FastSerialPort0(Serial);        // FTDI/console
FastSerialPort1(Serial1);       // GPS port
FastSerialPort3(Serial3);       // Telemetry port

////////////////////////////////////////////
// RC Hardware
////////////////////////////////////////////
#if CONFIG_APM_HARDWARE == APM_HARDWARE_APM2
APM_RC_APM2 APM_RC;
#else
APM_RC_APM1 APM_RC;
#endif

AP_LeadFilter xLeadFilter;      // GPS lag filter

void setup()
{
    Serial.begin(115200);
    Serial.println("ArduPilot RC Channel test");

    delay(500);

    int32_t temp = xLeadFilter.get_position(0, 100);
    Serial.printf("temp %ld \n", temp);
}

void loop()
{
}




