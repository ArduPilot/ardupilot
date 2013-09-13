
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Menu.h>
#include <AP_Param.h>
#include <AP_GPS.h>             // ArduPilot GPS library
#include <AP_ADC.h>             // ArduPilot Mega Analog to Digital Converter Library
#include <AP_Baro.h>
#include <AP_Declination.h>
#include <AP_Compass.h>         // ArduPilot Mega Magnetometer Library
#include <AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <AP_Curve.h>           // Curve used to linearlise throttle pwm to thrust
#include <AP_InertialSensor.h>  // ArduPilot Mega Inertial Sensor (accel & gyro) Library
                                // (only included for makefile libpath to work)
#include <AP_AHRS.h>
#include <APM_PI.h>             // PI library
#include <AC_PID.h>             // PID library
#include <RC_Channel.h>         // RC Channel Library
#include <AP_Motors.h>          // AP Motors library
#include <AP_ADC_AnalogSource.h>
#include <AP_RangeFinder.h>     // Range finder library
#include <AP_OpticalFlow.h>     // Optical Flow library
#include <Filter.h>             // Filter library
#include <AP_Buffer.h>          // APM FIFO Buffer
#include <AP_Relay.h>           // APM relay
#include <AP_Camera.h>          // Photo or video camera
#include <AP_Mount.h>           // Camera/Antenna mount
#include <AP_Airspeed.h>        // needed for AHRS build
#include <AP_Vehicle.h>         // needed for AHRS build
#include <AP_Notify.h>
#include <DataFlash.h>
#include <AP_InertialNav.h>     // ArduPilot Mega inertial navigation library
#include <GCS_MAVLink.h>
#include <memcheck.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM1
const AP_HAL::HAL& hal = AP_HAL_AVR_APM1;
#endif


void stream_loopback(AP_HAL::Stream* s, uint32_t time) {
    uint32_t end = hal.scheduler->millis() + time;
    for(;;) {
        if (hal.scheduler->millis() >= end && time != 0) {
            return;
        }
        if (s->available() > 0) {
            int c;
            c = s->read();
            if (-1 != c) {
                s->write((uint8_t)c);
            }
        }
    }
}

void stream_console_loopback(AP_HAL::Stream* s, AP_HAL::ConsoleDriver* c,
        uint32_t time) {
    uint32_t end = hal.scheduler->millis() + time;
    for(;;) {
        if (hal.scheduler->millis() >= end && time != 0) {
            return;
        }

        /* Read the uart, write to the console backend. */
        if (s->available() > 0) {
            int b;
            b = s->read();
            if (-1 != b) {
                uint8_t tmp[1];
                tmp[0] = (uint8_t) b;
                c->backend_write(tmp, 1);
            }
        }
        /* Loop back the console upon itself. */
        {
            int b;
            b = c->read();
            if (-1 != b) {
                c->write((uint8_t)b);
            }

        }
   
        /* Read the console backend, print to the uart. */
        { 
            uint8_t tmp[1];
            int readback = c->backend_read(tmp, 1);
            if (readback > 0) {
                s->write(tmp[0]);
            }

        }
    }
}
void setup(void)
{
        //
        // Test printing things
        //
    hal.console->print("test");
    hal.console->println(" begin");
    hal.console->println(1000);
    hal.console->println(1000, 8);
    hal.console->println(1000, 10);
    hal.console->println(1000, 16);
    hal.console->println_P(PSTR("progmem"));
    hal.console->printf("printf %d %u %#x %p %f %S\n", -1000, 1000, 1000, 1000, 1.2345, PSTR("progmem"));
    hal.console->printf_P(PSTR("printf_P %d %u %#x %p %f %S\n"), -1000, 1000, 1000, 1000, 1.2345, PSTR("progmem"));

    hal.console->println("loopback for 10sec:");
    stream_loopback(hal.console, 10000);
    hal.console->println("loopback done");

    hal.console->println("opening backend:");
    
    hal.console->backend_open();

    const char hello[] = "hello world\r\n";
    hal.console->backend_write((const uint8_t*)hello, strlen(hello));

    hal.console->println("loopback for 10sec:");
    stream_console_loopback(hal.console, hal.console, 10000);
    hal.console->println("loopback done");

    hal.console->backend_close();
    hal.console->println("closed backend.");

    hal.console->println("done.");
    for(;;);

}


void loop(void){}

AP_HAL_MAIN();
