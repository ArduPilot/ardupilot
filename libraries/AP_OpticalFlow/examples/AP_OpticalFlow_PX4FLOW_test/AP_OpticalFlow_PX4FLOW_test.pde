/*
 *  Example of AP_OpticalFlow library.
 *  Code by Randy Mackay. DIYDrones.com
 */

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <StorageManager.h>
#include <AP_Math.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_AHRS.h>
#include <AP_Compass.h>
#include <AP_Declination.h>
#include <AP_Airspeed.h>
#include <GCS_MAVLink.h>
#include <AP_Vehicle.h>
#include <AP_Notify.h>
#include <DataFlash.h>
#include <AP_GPS.h>
#include <AP_GPS_Glitch.h>      // GPS glitch protection library
#include <AP_InertialSensor.h>
#include <AP_ADC.h>
#include <AP_ADC_AnalogSource.h>
#include <Filter.h>
#include <AP_Baro.h>
#include <AP_OpticalFlow.h>
#include <AP_Mission.h>
#include <AP_Terrain.h>
#include <AP_NavEKF.h>
#include <AP_HAL_Linux.h>
#include <AP_HAL_Empty.h>
#include <AP_Rally.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

AP_InertialSensor ins;
AP_Baro baro;

// GPS declaration
static AP_GPS  gps;
GPS_Glitch gps_glitch(gps);

//AP_Compass_HMC5843 compass;
AP_AHRS_DCM ahrs(ins, baro, gps);

OpticalFlow optflow;

void setup()
{
    hal.console->println("OpticalFlow library test");

    hal.scheduler->delay(1000);

    // flowSensor initialization
    optflow.init();
    optflow.update();
    if (!optflow.healthy()) {
        hal.console->println("Failed to initialise OpticalFlow");
    }

    hal.scheduler->delay(1000);
}

//
// display menu
//
void display_menu()
{
    hal.console->println();
    hal.console->println("please choose from the following options:");
    hal.console->println("     d - display motion data");
    hal.console->println();
}

//
// display motion - show x,y and squal values constantly until user presses a key
//
void display_motion()
{
    bool first_time = true;

    // display instructions on how to exit
    //hal.console->println("press x to return to menu..");
    hal.scheduler->delay(1000);

    while (!hal.console->available()) {        
        
        // Update the measurement
        optflow.update();        

        // x,y,squal
        hal.console->printf_P(PSTR("device_id: %d\n"), (uint8_t)optflow.device_id());
        hal.console->printf_P(PSTR("surface_quality: %d\n"), (uint8_t)optflow.quality());
        hal.console->printf_P(PSTR("flowRate: x: %f y: %f\n"), (float)optflow.flowRate().x, (float)optflow.flowRate().y);
        hal.console->printf_P(PSTR("bodyRate: x: %f y: %f\n"), (float)optflow.bodyRate().x, (float)optflow.bodyRate().y);

        // short delay
        hal.scheduler->delay(100);
    }
}

void loop()
{

    hal.scheduler->delay(100);
    optflow.update();
    if (!optflow.healthy()) {
        hal.console->println("OpticalFlow is not healthy");
    }    
    display_motion();

    // int value;
    // display_menu();
    // // wait for user to enter something
    // while( !hal.console->available() ) {
    //     hal.scheduler->delay(20);
    // }

    // // get character from user
    // value = hal.console->read();

    // switch( value ) {

    // case 'd':
    //     display_motion();
    //     break;

    // default:
    //     hal.console->println("unrecognised command");
    //     hal.console->println();
    //     break;
    // }
}

AP_HAL_MAIN();
