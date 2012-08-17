/*
 *       Example of PID library.
 *       2010 Code by Jason Short. DIYDrones.com
 */

#include <FastSerial.h>
#include <avr/pgmspace.h>
#include <AP_Common.h>
#include <AP_Math.h>
#include <APM_RC.h> // ArduPilot RC Library
#include <PID.h> // ArduPilot Mega RC Library
#include <Arduino_Mega_ISR_Registry.h>

long radio_in;
long radio_trim;

Arduino_Mega_ISR_Registry isr_registry;

#if CONFIG_APM_HARDWARE == APM_HARDWARE_APM2
APM_RC_APM2 APM_RC;
#else
APM_RC_APM1 APM_RC;
#endif

PID pid;

void setup()
{
    Serial.begin(38400);
    Serial.println("ArduPilot Mega PID library test");
    APM_RC.Init(&isr_registry);                 // APM Radio initialization

    delay(1000);
    //rc.trim();
    radio_trim = APM_RC.InputCh(0);

    pid.kP(1);
    pid.kI(0);
    pid.kD(0.5);
    pid.imax(50);
    pid.save_gains();
    pid.kP(0);
    pid.kI(0);
    pid.kD(0);
    pid.imax(0);
    pid.load_gains();
    Serial.printf("P %f  I %f  D %f  imax %f\n", pid.kP(), pid.kI(), pid.kD(), pid.imax());
}

void loop()
{
    delay(20);
    //rc.read_pwm();
    long error              = APM_RC.InputCh(0) - radio_trim;
    long control    = pid.get_pid(error, 1);

    Serial.print("control: ");
    Serial.println(control,DEC);
}
