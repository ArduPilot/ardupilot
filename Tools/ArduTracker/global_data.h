// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#ifndef __GLOBAL_DATA_H
#define __GLOBAL_DATA_H

#include "param_table.h"

#define ONBOARD_PARAM_NAME_LENGTH 15
#define MAX_WAYPOINTS 20

#define FIRMWARE_VERSION 12 // <-- change on param data struct modifications

#ifdef __cplusplus

///
// global data structure
// This structure holds all the vehicle parameters.
// TODO: bring in varibles floating around in ArduPilotMega.pde
//
struct global_struct
{
	// parameters
	uint16_t parameter_i; // parameter index
	uint16_t param_count; // number of params

	// waypoints
	uint16_t waypoint_request_i; // request index
	uint16_t waypoint_dest_sysid; // where to send requests
	uint16_t waypoint_dest_compid; // "
	bool waypoint_sending; // currently in send process
	bool waypoint_receiving; // currently receiving
	uint16_t waypoint_count;
	uint32_t waypoint_timelast_send; // milliseconds
	uint32_t waypoint_timelast_receive; // milliseconds
	uint16_t waypoint_send_timeout; // milliseconds
	uint16_t waypoint_receive_timeout; // milliseconds
	float junk; //used to return a junk value for interface

	// data stream rates
	uint16_t streamRateRawSensors;
	uint16_t streamRateExtendedStatus;
	uint16_t streamRateRCChannels;
	uint16_t streamRateRawController;
	uint16_t streamRateRawSensorFusion;
	uint16_t streamRatePosition;
	uint16_t streamRateExtra1;
	uint16_t streamRateExtra2;
	uint16_t streamRateExtra3;

	// struct constructor
	global_struct();
} global_data;

extern "C" const char *param_nametab[];

#endif // __cplusplus

#endif // __GLOBAL_DATA_H
