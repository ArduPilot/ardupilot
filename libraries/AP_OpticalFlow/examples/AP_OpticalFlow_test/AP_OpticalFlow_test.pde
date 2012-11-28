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

const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;

AP_OpticalFlow_ADNS3080 flowSensor;

void setup()
{
    hal.console->println("ArduPilot Mega OpticalFlow library test ver 1.5");

    hal.scheduler->delay(1000);

    // flowSensor initialization
    if( flowSensor.init() == false ) {
        hal.console->print("Failed to initialise ADNS3080 ");
    }

    flowSensor.set_orientation(AP_OPTICALFLOW_ADNS3080_PINS_FORWARD);
    flowSensor.set_field_of_view(AP_OPTICALFLOW_ADNS3080_08_FOV);

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
    hal.console->println("     f - set frame rate");
    hal.console->println("     i - display image");
    hal.console->println("     I - display image continuously");
    hal.console->println("     m - display motion");
    hal.console->println("     r - set resolution");
    hal.console->println("     s - set shutter speed");
    hal.console->println("     z - clear all motion");
    hal.console->println("     a - frame rate auto/manual");
    hal.console->println();
}

//
// display config
//
void display_config()
{
    hal.console->print("Config: ");
    hal.console->print(flowSensor.read_register(ADNS3080_CONFIGURATION_BITS),BIN);
    hal.scheduler->delay_microseconds(50);
    hal.console->print(",");
    hal.console->print(flowSensor.read_register(ADNS3080_EXTENDED_CONFIG),BIN);
    hal.scheduler->delay_microseconds(50);
    hal.console->println();

    // product id
    hal.console->print("\tproduct id:     ");
    hal.console->print(flowSensor.read_register(ADNS3080_PRODUCT_ID),HEX);
    hal.scheduler->delay_microseconds(50);
    hal.console->print(" (hex)");
    hal.console->println();

    // frame rate
    hal.console->print("\tframe rate:     ");
    hal.console->print(flowSensor.get_frame_rate());
    if( flowSensor.get_frame_rate_auto() == true ) {
        hal.console->print(" (auto)");
    }else{
        hal.console->print(" (manual)");
    }
    hal.console->println();

    // resolution
    hal.console->print("\tresolution:     ");
    hal.console->print(flowSensor.get_resolution());
    hal.console->println();

    // shutter speed
    hal.console->print("\tshutter speed:  ");
    hal.console->print(flowSensor.get_shutter_speed());
    if( flowSensor.get_shutter_speed_auto() ) {
        hal.console->print(" (auto)");
    }else{
        hal.console->print(" (manual)");
    }
    hal.console->println();
}

//
// set frame rate
//
void set_frame_rate()
{
    int value;

    // frame rate
    hal.console->print("frame rate:     ");
    hal.console->print(flowSensor.get_frame_rate());
    if( flowSensor.get_frame_rate_auto() == true ) {
        hal.console->print(" (auto)");
    }else{
        hal.console->print(" (manual)");
    }
    hal.console->println();

    hal.console->println("Choose new frame rate:");
    hal.console->println("\ta) auto");
    hal.console->println("\t2) 2000 f/s");
    hal.console->println("\t3) 3000 f/s");
    hal.console->println("\t4) 4000 f/s");
    hal.console->println("\t5) 5000 f/s");
    hal.console->println("\t6) 6400 f/s");
    hal.console->println("\tx) exit (leave unchanged)");

    // get user input
    //hal.console->flush();
    while( !hal.console->available() ) {
        hal.scheduler->delay(20);
    }
    value = hal.console->read();

    if( value == 'a' || value == 'A')
        flowSensor.set_frame_rate_auto(true);
    if( value == '2' )
        flowSensor.set_frame_rate(2000);
    if( value == '3' )
        flowSensor.set_frame_rate(3000);
    if( value == '4' )
        flowSensor.set_frame_rate(4000);
    if( value == '5' )
        flowSensor.set_frame_rate(5000);
    if( value == '6' )
        flowSensor.set_frame_rate(6469);

    // display new frame rate
    hal.console->print("frame rate:     ");
    hal.console->print(flowSensor.get_frame_rate());
    if( flowSensor.get_frame_rate_auto() == true ) {
        hal.console->print(" (auto)");
    }else{
        hal.console->print(" (manual)");
    }
    hal.console->println();
}

// display_image - captures and displays image from flowSensor flowSensor
void display_image()
{
    hal.console->println("image data --------------");
    flowSensor.print_pixel_data();
    hal.console->println("-------------------------");
}

// display_image - captures and displays image from flowSensor flowSensor
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

    //hal.console->flush();
}

//
// set resolutiojn
//
void set_resolution()
{
    int value;
    int resolution = flowSensor.get_resolution();
    hal.console->print("resolution: ");
    hal.console->println(resolution);
    hal.console->println("Choose new value:");
    hal.console->println("    1) 1600");
    hal.console->println("    4) 400");
    hal.console->println("    x) exit");
    hal.console->println();

    // get user input
    //hal.console->flush();
    while( !hal.console->available() ) {
        hal.scheduler->delay(20);
    }
    value = hal.console->read();

    // update resolution
    if( value == '1' ) {
        flowSensor.set_resolution(ADNS3080_RESOLUTION_1600);
    }
    if( value == '4' ) {
        flowSensor.set_resolution(ADNS3080_RESOLUTION_400);
    }

    hal.console->print("new resolution: ");
    hal.console->println(flowSensor.get_resolution());
}

//
// set shutter speed
//
void set_shutter_speed()
{
    int value;

    // shutter speed
    hal.console->print("shutter speed:     ");
    hal.console->print(flowSensor.get_shutter_speed());
    if( flowSensor.get_shutter_speed_auto() == true ) {
        hal.console->print(" (auto)");
    }else{
        hal.console->print(" (manual)");
    }
    hal.console->println();

    hal.console->println("Choose new shutter speed:");
    hal.console->println("\ta) auto");
    hal.console->println("\t1) 1000 clock cycles");
    hal.console->println("\t2) 2000 clock cycles");
    hal.console->println("\t3) 3000 clock cycles");
    hal.console->println("\t4) 4000 clock cycles");
    hal.console->println("\t5) 5000 clock cycles");
    hal.console->println("\t6) 6000 clock cycles");
    hal.console->println("\t7) 7000 clock cycles");
    hal.console->println("\t8) 8000 clock cycles");
    hal.console->println("\t9) 9000 clock cycles");
    hal.console->println("\tx) exit (leave unchanged)");

    // get user input
    //hal.console->flush();
    while( !hal.console->available() ) {
        hal.scheduler->delay(20);
    }
    value = hal.console->read();

    if( value == 'a' || value == 'A')
        flowSensor.set_shutter_speed_auto(true);
    if( value == '1' )
        flowSensor.set_shutter_speed(1000);
    if( value == '2' )
        flowSensor.set_shutter_speed(2000);
    if( value == '3' )
        flowSensor.set_shutter_speed(3000);
    if( value == '4' )
        flowSensor.set_shutter_speed(4000);
    if( value == '5' )
        flowSensor.set_shutter_speed(5000);
    if( value == '6' )
        flowSensor.set_shutter_speed(6000);
    if( value == '7' )
        flowSensor.set_shutter_speed(7000);
    if( value == '8' )
        flowSensor.set_shutter_speed(8000);
    if( value == '9' )
        flowSensor.set_shutter_speed(9000);

    // display new shutter speed
    hal.console->print("shutter speed:     ");
    hal.console->print(flowSensor.get_shutter_speed());
    if( flowSensor.get_shutter_speed_auto() == true ) {
        hal.console->print(" (auto)");
    }else{
        hal.console->print(" (manual)");
    }
    hal.console->println();
}

//
// display motion - show x,y and squal values constantly until user presses a key
//
void display_motion()
{
    bool first_time = true;
    //hal.console->flush();

    // display instructions on how to exit
    hal.console->println("press x to return to menu..");
    hal.scheduler->delay(1000);

    while( !hal.console->available() ) {
        //flowSensor.update();
        flowSensor.update_position(0,0,0,1,100);

        // check for errors
        if( flowSensor.overflow() )
            hal.console->println("overflow!!");

        // x,y,squal
        hal.console->print("x/dx: ");
        hal.console->print(flowSensor.x,DEC);
        hal.console->print("/");
        hal.console->print(flowSensor.dx,DEC);
        hal.console->print("\ty/dy: ");
        hal.console->print(flowSensor.y,DEC);
        hal.console->print("/");
        hal.console->print(flowSensor.dy,DEC);
        hal.console->print("\tsqual:");
        hal.console->print(flowSensor.surface_quality,DEC);
        hal.console->println();
        first_time = false;

        // short delay
        hal.scheduler->delay(100);
    }

    // flush the serial
    //hal.console->flush();
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

    case 'f':
        // set frame rate
        set_frame_rate();
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

    case 'r':
        // set resolution
        set_resolution();
        break;

    case 's':
        // set shutter speed
        set_shutter_speed();
        break;

    case 'z':
        // clear and reset everything
        flowSensor.clear_motion();
        break;

    default:
        hal.console->println("unrecognised command");
        hal.console->println();
        break;
    }
}

AP_HAL_MAIN();
