/*
 *  Example of AP_Mission Library.
 *  DIYDrones.com
 */

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <AP_Curve.h>           // Curve used to linearlise throttle pwm to thrust
#include <AP_Param.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <GCS_MAVLink.h>
#include <AP_Notify.h>
#include <AP_Vehicle.h>
#include <DataFlash.h>
#include <AP_Mission.h>
#include <AP_GPS.h>             // ArduPilot GPS library
#include <AP_GPS_Glitch.h>      // GPS glitch protection library
#include <AP_ADC.h>             // ArduPilot Mega Analog to Digital Converter Library
#include <AP_ADC_AnalogSource.h>
#include <AP_Baro.h>            // ArduPilot Mega Barometer Library
#include <Filter.h>
#include <AP_Compass.h>         // ArduPilot Mega Magnetometer Library
#include <AP_Declination.h>
#include <AP_InertialSensor.h>  // ArduPilot Mega Inertial Sensor (accel & gyro) Library
#include <AP_AHRS.h>
#include <AP_Airspeed.h>
#include <AP_Buffer.h>          // ArduPilot general purpose FIFO buffer
#include <GCS_MAVLink.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

// start_cmd - function that is called when new command is started
//      should return true if command is successfully started
bool start_cmd(const AP_Mission::Mission_Command& cmd)
{
    hal.console->printf_P(PSTR("started cmd #%d id:%d\n"),(int)cmd.index, (int)cmd.id);
    return true;
}

// verify_mcd - function that is called repeatedly to ensure a command is progressing
//      should return true once command is completed
bool verify_cmd(const AP_Mission::Mission_Command& cmd)
{
    hal.console->printf_P(PSTR("Verified cmd #%d id:%d\n"),(int)cmd.index,(int)cmd.id);
    return true;
}

// mission_complete - function that is called once the mission completes
void mission_complete(void)
{
    hal.console->printf_P(PSTR("mission complete function called!\n"));
}

// declaration
AP_Mission mission(&start_cmd, &verify_cmd, &mission_complete);

// setup
void setup(void)
{
    hal.console->println("AP_Mission library test\n");

    // display basic info about command sizes
    hal.console->printf_P(PSTR("Max Num Commands: %d\n"),(int)AP_MISSION_MAX_COMMANDS);
    hal.console->printf_P(PSTR("Command size: %d bytes\n"),(int)AP_MISSION_EEPROM_COMMAND_SIZE);
    hal.console->printf_P(PSTR("Command start in Eeprom: %x\n"),(int)AP_MISSION_EEPROM_START_BYTE);
}

// loop
void loop(void)
{
    // initialise mission
    init_mission();

    // print current mission
    print_mission();

    // start mission
    hal.console->printf_P(PSTR("\nRunning missions\n"));
    mission.start();

    // update mission until it completes
    while(!mission.state() != AP_Mission::MISSION_COMPLETE) {
        mission.update();
    }
    hal.console->printf_P(PSTR("Mission Complete!\n"));

    // wait forever
    while(true);
}

// init_mission - initialise the mission to hold something
void init_mission()
{
    AP_Mission::Mission_Command cmd;

    // clear mission
    mission.clear();

    // Command #0 : take-off to 10m
    cmd.id = MAV_CMD_NAV_TAKEOFF;
    cmd.content.location.options = 0;
    cmd.content.location.p1 = 0;
    cmd.content.location.alt = 10;
    cmd.content.location.lat = 0;
    cmd.content.location.lng = 0;
    if (!mission.add_cmd(cmd)) {
        hal.console->printf_P(PSTR("failed to add command\n"));
    }

    // Command #1 : first waypoint
    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.content.location.options = 0;
    cmd.content.location.p1 = 0;
    cmd.content.location.alt = 11;
    cmd.content.location.lat = 12345678;
    cmd.content.location.lng = 23456789;
    if (!mission.add_cmd(cmd)) {
        hal.console->printf_P(PSTR("failed to add command\n"));
    }

    // Command #2 : second waypoint
    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.content.location.p1 = 0;
    cmd.content.location.lat = 1234567890;
    cmd.content.location.lng = -1234567890;
    cmd.content.location.alt = 22;
    if (!mission.add_cmd(cmd)) {
        hal.console->printf_P(PSTR("failed to add command\n"));
    }

    // Command #3 : do-jump to first waypoint 3 times
    cmd.id = MAV_CMD_DO_JUMP;
    cmd.content.jump.target = 1;
    cmd.content.jump.num_times = 1;
    if (!mission.add_cmd(cmd)) {
        hal.console->printf_P(PSTR("failed to add command\n"));
    }
    cmd.index = 3;

    // Command #4 : RTL
    cmd.id = MAV_CMD_NAV_RETURN_TO_LAUNCH;
    cmd.content.location.p1 = 0;
    cmd.content.location.lat = 0;
    cmd.content.location.lng = 0;
    cmd.content.location.alt = 0;
    if (!mission.add_cmd(cmd)) {
        hal.console->printf_P(PSTR("failed to add command\n"));
    }
}

// print_mission - print out the entire mission to the console
void print_mission()
{
    AP_Mission::Mission_Command cmd;

    // check for empty mission
    if (mission.num_commands() == 0) {
        hal.console->printf_P(PSTR("No Mission!\n"));
        return;
    }

    hal.console->printf_P(PSTR("Mission: %d commands\n"),(int)mission.num_commands());

    // print each command
    for(uint16_t i=0; i<mission.num_commands(); i++) {
        // get next command from eeprom
        mission.read_cmd_from_storage(i,cmd);

        // print command position in list and mavlink id
        hal.console->printf_P(PSTR("Cmd#%d mav-id:%d "), (int)cmd.index, (int)cmd.id);

        // print command contents
        if (cmd.id == MAV_CMD_DO_JUMP) {
            hal.console->printf_P(PSTR("jump-to:%d num_times:%d\n"), (int)cmd.content.jump.target, (int)cmd.content.jump.num_times);
        }else{
            hal.console->printf_P(PSTR("p1:%d lat:%ld lng:%ld alt:%ld\n"),(int)cmd.content.location.p1, (long)cmd.content.location.lat, (long)cmd.content.location.lng, (long)cmd.content.location.alt);
        }
    }
    hal.console->printf_P(PSTR("--------\n"));
}

AP_HAL_MAIN();
