/*
 *  Example of AP_OpticalFlow library.
 *  Code by Randy Mackay. DIYDrones.com
 */

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

#include <AP_OpticalFlow.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

AP_OpticalFlow_ADNS3080 optflow;

void setup()
{
    hal.console->println("ArduPilot Mega OpticalFlow library test ver 1.5");

    hal.scheduler->delay(1000);

    // flowSensor initialization
    optflow.init();
    if (!optflow.healthy()) {
        hal.console->print("Failed to initialise ADNS3080 ");
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
    hal.console->println("     c - display all config");
    hal.console->println("     i - display image");
    hal.console->println("     I - display image continuously");
    hal.console->println("     m - display motion");
    hal.console->println();
}

//
// display config
//
void display_config()
{
    // product id
    hal.console->printf_P(PSTR("Product id: %#x\n"),optflow.read_register(ADNS3080_PRODUCT_ID));
    hal.scheduler->delay_microseconds(50);

    uint8_t reg = optflow.read_register(ADNS3080_CONFIGURATION_BITS);
    hal.console->printf_P(PSTR("Config registers: %#x"),optflow.read_register(ADNS3080_CONFIGURATION_BITS));
    hal.scheduler->delay_microseconds(50);
    hal.console->printf_P(PSTR(",%#x\n"),optflow.read_register(ADNS3080_EXTENDED_CONFIG));
    hal.scheduler->delay_microseconds(50);
}

// display_image - captures and displays image from flowSensor flowSensor
void display_image()
{
    hal.console->println("image data --------------");
    optflow.print_pixel_data();
    hal.console->println("-------------------------");
}

// display_image - captures and displays image from flowSensor
void display_image_continuously()
{
    int i;
    hal.console->println("press any key to return to menu");

    //hal.console->flush();

    while( !hal.console->available() ) {
        display_image();
        i=0;
        while( i<20 && !hal.console->available() ) {
            hal.scheduler->delay(100);          // give the viewer a bit of time to catchup
            i++;
        }
    }
}

//
// display motion - show x,y and squal values constantly until user presses a key
//
void display_motion()
{
    bool first_time = true;

    // display instructions on how to exit
    hal.console->println("press x to return to menu..");
    hal.scheduler->delay(1000);

    while (!hal.console->available()) {
        optflow.update_position(0,0,0,1,100);

        // x,y,squal
        hal.console->printf_P(PSTR("x_cm/dx:%d/%d\ty_cm/dy:%d/%d\tsqual:%d\n"),(int)optflow.x_cm,(int)optflow.dx,(int)optflow.y_cm,(int)optflow.dy,(int)optflow.surface_quality);
        first_time = false;

        // short delay
        hal.scheduler->delay(100);
    }
}

void loop()
{
    int value;

    // display menu to user
    display_menu();

    // wait for user to enter something
    while( !hal.console->available() ) {
        hal.scheduler->delay(20);
    }

    // get character from user
    value = hal.console->read();

    switch( value ) {

    case 'c':
        // display all config
        display_config();
        break;

    case 'i':
        // display image
        display_image();
        break;

    case 'I':
        // display image continuously
        display_image_continuously();
        break;

    case 'm':
        // display motion
        display_motion();
        break;

    default:
        hal.console->println("unrecognised command");
        hal.console->println();
        break;
    }
}

AP_HAL_MAIN();
