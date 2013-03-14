// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// This file is just a placeholder for your configuration file.  If
// you wish to change any of the setup parameters from their default
// values, place the appropriate #define statements here.

// If you used to define your CONFIG_APM_HARDWARE setting here, it is no
// longer valid! You should switch to using CONFIG_HAL_BOARD via the HAL_BOARD
// flag in your local config.mk instead.

// The following are the recommended settings for Xplane
// simulation. Remove the leading "/* and trailing "*/" to enable:

//#define HIL_MODE            HIL_MODE_DISABLED

/*
 *  // HIL_MODE SELECTION
 *  //
 *  // Mavlink supports
 *  // 1. HIL_MODE_ATTITUDE : simulated position, airspeed, and attitude
 *  // 2. HIL_MODE_SENSORS: full sensor simulation
 *  //#define HIL_MODE            HIL_MODE_ATTITUDE
 *
 */

	/* TO enable telemetry and all interaircraft communication on 
	// UART2 (hal.uartC). Preferred this to hal.UARTA so that hal.uartA
	// could be used for running HIL simulations through USB
	*/	
# define SERIAL3_BAUD                    57600
# define TELEMETRY_UART2 ENABLED

# define HIL_MODE            HIL_MODE_ATTITUDE
//# define HIL_MODE            HIL_MODE_DISABLED

//channel for mode switch
# define FLIGHT_MODE_CHANNEL    5

// new APM_Control controller library by Jon Challinger
//# define APM_CONTROL ENABLED

//////////////////////////////////////////////////////////////////////////////
//Cooperative Flight setup
//////////////////////////////////////////////////////////////////////////////

	/* Disable this to prevent any interaircraft data to be sent or recieved.
	// Note mavlink_helpers.h have their own # defines to enable Xbee API communication
	*/
#ifndef COOPERATIVE_MISSION
 # define COOPERATIVE_MISSION ENABLED
#endif

	/* To set this aircraft as leader, it'll keep sending location data to follower
	// at all times indifferent of its control mode
	*/
#ifndef AIRCRAFT_TYPE_LEADER
 # define AIRCRAFT_TYPE_LEADER DISABLED
#endif


	/* To set this aircraft as follower. It'll receive leader location data and followe 
	// ONLY in guided mode. Switch using RC controller or GCS. An aircraft can be leader
	// and follower simulataneously. ex LEADER(MAV1)>>----<<FOLLOWER (MAV2) LEADER >>----<<FOLLOWER(MAV3) 
    */
#ifndef AIRCRAFT_TYPE_FOLLOWER
 # define AIRCRAFT_TYPE_FOLLOWER ENABLED
#endif

	/* To set this aircraft as an Orbit follower. Only enabled in guided mode, the aircraft
	// never goes to the guided waypoint but circles around it. Also, it shares the orbit center data
	// with any other aircraft in the orbit follower mode. So, set orbit center for any aircraft using 
	// GCS and all other aircrafts automatically flock to circle around that center. 		
	*/
#ifndef AIRCRAFT_TYPE_ORBIT_FOLLOWER
 # define AIRCRAFT_TYPE_ORBIT_FOLLOWER DISABLED
#endif
