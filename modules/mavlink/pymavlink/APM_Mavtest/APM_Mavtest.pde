/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/*
  send all possible mavlink messages
  Andrew Tridgell July 2011
*/

// AVR runtime
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <math.h>

// Libraries
#include <FastSerial.h>
#include <AP_Common.h>
#include <GCS_MAVLink.h>
#include "mavtest.h"

FastSerialPort0(Serial);        // FTDI/console
FastSerialPort1(Serial1);       // GPS port
FastSerialPort3(Serial3);       // Telemetry port

#define SERIAL0_BAUD 115200
#define SERIAL3_BAUD 115200

void setup() {
	Serial.begin(SERIAL0_BAUD, 128, 128);
	Serial3.begin(SERIAL3_BAUD, 128, 128);
	mavlink_comm_0_port = &Serial;
	mavlink_comm_1_port = &Serial3;
}



void loop()
{
	Serial.println("Starting MAVLink test generator\n");
	while (1) {
		mavlink_msg_heartbeat_send(
			MAVLINK_COMM_0,
			mavlink_system.type,
			MAV_AUTOPILOT_ARDUPILOTMEGA);

		mavlink_msg_heartbeat_send(
			MAVLINK_COMM_1,
			mavlink_system.type,
			MAV_AUTOPILOT_ARDUPILOTMEGA);

		mavtest_generate_outputs(MAVLINK_COMM_0);
		mavtest_generate_outputs(MAVLINK_COMM_1);
		delay(500);
	}
}

