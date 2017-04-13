/*
 *       Example of PID library.
 *       2010 Code by Jason Short. DIYDrones.com
 */

#include <AP_HAL/AP_HAL.h>
#include <PID/PID.h> // ArduPilot Mega RC Library

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

long radio_in;
long radio_trim;

PID pid;

void setup()
{
    hal.console->printf("ArduPilot Mega PID library test\n");

    hal.scheduler->delay(1000);
    //rc.trim();
    radio_trim = hal.rcin->read(0);

    pid.kP(1);
    pid.kI(0);
    pid.kD(0.5f);
    pid.imax(50);
    pid.save_gains();
    pid.kP(0);
    pid.kI(0);
    pid.kD(0);
    pid.imax(0);
    pid.load_gains();
    hal.console->printf(
            "P %f  I %f  D %f  imax %d\n",
            (double)pid.kP(),
            (double)pid.kI(),
            (double)pid.kD(), pid.imax());
}

void loop()
{
    hal.scheduler->delay(20);
    //rc.read_pwm();
    long error  = hal.rcin->read(0) - radio_trim;
    long control= pid.get_pid(error, 1);

    hal.console->printf("control: ");
    hal.console->printf("%ld\n", control);
}

AP_HAL_MAIN();
