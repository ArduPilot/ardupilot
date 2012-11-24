/*
 *       Example of PID library.
 *       2010 Code by Jason Short. DIYDrones.com
 */

#include <Arduino_Mega_ISR_Registry.h>
#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Math.h>
#include <APM_RC.h> // ArduPilot RC Library
#include <AP_PID.h> // ArduPilot Mega RC Library

long radio_in;
long radio_trim;

Arduino_Mega_ISR_Registry isr_registry;
AP_PID pid;
APM_RC_APM1 APM_RC;

void setup()
{
    Serial.begin(115200);
    Serial.println("ArduPilot Mega AP_PID library test");

    isr_registry.init();
    APM_RC.Init(&isr_registry);          // APM Radio initialization

    delay(1000);
    //rc.trim();
    radio_trim = APM_RC.InputCh(0);

    pid.kP(1);
    pid.kI(0);
    pid.kD(0.5);
    pid.imax(50);
    pid.kP(0);
    pid.kI(0);
    pid.kD(0);
    pid.imax(0);
    Serial.printf("P %f  I %f  D %f  imax %f\n", pid.kP(), pid.kI(), pid.kD(), pid.imax());
}

void loop()
{
    delay(20);
    //rc.read_pwm();
    long error              = APM_RC.InputCh(0) - radio_trim;
    long control    = pid.get_pid(error, 20, 1);

    Serial.print("control: ");
    Serial.println(control,DEC);
}
