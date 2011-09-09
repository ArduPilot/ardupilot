/*
Arducopter 2 Waypoint writer
Use this release to manually upload waypoints
*/
#include "defines.h"
#include <FastSerial.h>
#include <AP_Common.h>
#include <GCS_MAVLink.h>    // MAVLink GCS definitions

FastSerialPort0(Serial);        // FTDI/console

// EEPROM addresses
#define EEPROM_MAX_ADDR		4096
// parameters get the first 1KiB of EEPROM, remainder is for waypoints
#define WP_START_BYTE 0x400 // where in memory home WP is stored + all other WP
#define WP_SIZE 15

// you can keep your missions stored here, just uncommment the mission to load: (only 1 at a time.)
//#include "mission_example.h"
#include "sparkfun.h"
//#include "takeoff.h"

//#include "SFO_T3.h"
//#include "SFO_landMe.h"
//#include "Basic_mission_example.h"

const long t7 = 10000000;	// used to scale GPS values for EEPROM storage

byte 	yaw_tracking = TRACK_NONE;			// no tracking, point at next wp, or at a target


#define k_param_RTL_altitude 163
#define k_param_waypoint_total 221
#define k_param_waypoint_radius 224

AP_Int16	RTL_altitude      ((int8_t)9,					k_param_RTL_altitude,     NULL);
AP_Int8		waypoint_total    ((int8_t)0,                   k_param_waypoint_total,   NULL);
AP_Int8		waypoint_radius   ((int8_t)WP_RADIUS,			k_param_waypoint_radius,  NULL);


// You DON'T need to edit below this line
// ----------------------------------
#include <avr/io.h>
#include <avr/eeprom.h>

void setup()
{
	Serial.begin(38400);
	delay(1000);
	Serial.println("\nACM Waypoint writer 1.0.3 Public Alpha \n\n");

	Serial.printf("Test: %d\n", (int)MAV_CMD_NAV_LAND);

	//*
	// number of waypoints, add 1 for home
	waypoint_total.set_and_save((sizeof(mission) / 24));
	waypoint_radius.set_and_save(WP_RADIUS);
	RTL_altitude.set_and_save(ALT_TO_HOLD * 100);

	writePoints();  			// saves Waypoint Array

	delay(1000);

	if(RTL_altitude < 0){
		Serial.print("Hold current altitude above home after RTL.\n");
	}else{
		Serial.printf("Hold this altitude over home: %ld meters\n", (long)RTL_altitude.get());
	}

	Serial.printf("WP Radius: %ld meters\n", (long)RTL_altitude.get());
	Serial.printf("WP Radius: %d meters\n", (int)waypoint_radius.get());
	Serial.printf("total # of commands: %d\n", (int)waypoint_total.get());

	report_wp(255);

	//*/
}

void loop()
{

}


void report_wp(byte index)
{
	if(index == 255){
		for(byte i = 0; i <= waypoint_total; i++){
			struct Location temp = get_wp_with_index(i);
			print_wp(&temp, i);
		}
	}else{
		struct Location temp = get_wp_with_index(index);
		print_wp(&temp, index);
	}
}

void print_wp(struct Location *cmd, byte index)
{
	Serial.printf_P(PSTR("command #: %d \tid:%d\top:%d\tp1:%d\tp2:%ld\tp3:%ld\tp4:%ld \n"),
		(int)index,
		(int)cmd->id,
		(int)cmd->options,
		(int)cmd->p1,
		cmd->alt,
		cmd->lat,
		cmd->lng);
}
