# ArduPilot Project

## Getting the source

You can either download the source using the "ZIP" button at the top of the
github page, or you can make a clone using git:

```
git clone git://github.com/diydrones/ardupilot.git
```

## Building using Arduino IDE

ArduPilot is no longer compatible with the standard Arduino distribution.
You need to use a patched Arduino IDE to build ArduPilot.

1. The patched ArduPilot Arduino IDE is available for Mac and Windows from
   the [downloads page][1]. On Linux, you should use the makefile build.

2. Unpack and launch the ArduPilot Arduino IDE. In the preferences menu, set
   your sketchbook location to your downloaded or cloned `ardupilot` directory.

3. In the ArduPilot Arduino IDE, select your ArduPilot type (APM1 or APM2) from
   the ArduPilot menu (in the top menubar).

4. Restart the ArduPilot Arduino IDE. You should now be able to build ArduPlane
   or ArduCopter from source.

5. Remember that, after changing ArduPilot type (APM1 or APM2) in the IDE,
   you'll need to close and restart the IDE before continuing.

[1]: http://code.google.com/p/ardupilot-mega/downloads/list


## Building using make 

 1. Before you build the project for the first time, you'll need to run
    `make configure` from a  sketch directory. This will create a `config.mk`
    file at the top level of the repository. You can set some defaults in
    `config.mk`

 2. In the sketch directory, type `make` to build for APM2. Alternatively,
    `make apm1` will build for the APM1.

 3. Type `make upload` to upload. You may need to set the correct default
    serial port in your `config.mk`.

# User Technical Support

ArduPilot users should use the DIYDrones.com forums for technical support.

# Development Team

The ArduPilot project is open source and maintained by a team of volunteers.

To contribute, you can send a pull request on Github. You can also join the
[development discussion on Google Groups][2]. Note that the Google Groups
mailing lists are NOT for user tech support, and are moderated for new users to
prevent off-topic discussion.

[2]: https://groups.google.com/forum/?fromgroups#!forum/drones-discuss

# Cooperative Flight Using ArduPilot
Following is a brief set of instructions for setting up a cooperative flight framework using ArduPilot code. It has been 
tested in HIL and in flight. The code is tested to work on APM2, APM2.5, PX4fmu standalone and PX4IO. A brief change 
change-log from standard APM repo is also provided below. 

## Building On Linux
   1. Getting and compiling the default firmware.
	  1. Gitclone latest ArduPlane using git clone git://github.com/diydrones/ardupilot.git
	  2. Ardupilot->ArduPlane, in terminal 'make configure'
	  3. Modify path to px4 firmware(https://github.com/tridge/Firmware) root, uncomment it
	  4. In px4 firmware folder, in terminal 'make configure_px4fmu'
	  5. Verify default APM code,in ArduPlane, in terminal 'make', successful, 'make clean'
	  6. Verify for px4, in ArduPlane, in terminal 'make px4-clean', 'make px4', successful

	2. Changes to default firmware
	  1. In libraries folder:
  	    1. Add the folder APM_Xbee to the libraries folder in ardupilot
	    2. Replace the file GCS_MAVLink.h file in the folder libraries->GCS_MAVLink folder
	    3. Replace the file mavlink_helpers.h in the folder libraries->GCS_MAVLink->include->mavlink->v1.0 folder 
	  2. In ArduPlane folder:
	    1. Add the file follower_guided_wp.pde
	    2. Replace the file APM_Config.h
	    3. Replace the file navigation.pde
	    4. Replace the file ArduPlane.pde	

	3. To Compile, in terminal type 'make', compile sucessful. 'make px4-clean' & 'make px4'
		


##MODIFICATIONS:

In file GCS_MAVLink.h:

	List of Functions modified:
		/* Modified it to look for interaircraft xbee data start tag 0xAF 0xFB
		// and when found, store the next 13 bytes in a temp array, check for data 
		// crc using check_recieved_packet() and store the data prmanently 	
		*/
		static inline uint8_t comm_receive_ch(mavlink_channel_t chan){}


	List of Functions added:
		/* Function calculates the interaircraft data crc and checks it against
		// the one in the packet. Then appropriately stores the data.
		*/
		static inline void check_recieved_packet(void){}


	List of Variables added:
		static    uint8_t o_packet[14] = {0};//temp
		static    uint8_t W_packet[13] = {0};//temp

		static    uint8_t flag1 = 0;//flags for interrupt handlers
		static    uint8_t flag2 = 0;//flags for interrupt handlers
		static    uint8_t ctr = 0;//flags for interrupt handlers

		static    uint8_t ourpacket[13] = {0};
		static    uint8_t WPpacket[12] = {0}; //IMPORTANT: Stores the most recent correct interaircraft data
		static    uint8_t PWPpacket[12] = {0};//IMPORTANT: Stores the previous most recent correct interaircraft data

		static    uint8_t xbee_flag = 0;

		static    uint8_t evnt_WPpacket[12] = {0}; //IMPORTANT: Stores the most recent correct interaircraft data of orbit center
		static    uint8_t evnt_PWPpacket[12] = {0}; //IMPORTANT: Stores the previous most recent correct interaircraft data of  orbit center	

		#define FOLLOWER_WP_CRC 0xAA
		#define ORBIT_CENTER_CRC 0xBB


In file mavlink_helpers.h:

	List of Functions modified:
		/* #if API, it is modified to calculate and add API header and Footer for every mavlink 
		// packet to be sent to the groumdstation.
		*/
		MAVLINK_HELPER void _mav_finalize_message_chan_send(mavlink_channel_t chan, uint8_t msgid, const char *packet, uint8_t length)

		/* #if API, it is modified to add API header and Footer for every mavlink 
		// packet to be sent to the groumdstation.
		*/	
		MAVLINK_HELPER void _mavlink_resend_uart(mavlink_channel_t chan, const mavlink_message_t *msg)

	List of Functions added:
		None

	List of Variables added:
		#define API 1
		#define GCS_ADDR_HIGH 0x10  //Groundstation Xbee address
		#define GCS_ADDR_LOW  0x01  //Groundstation Xbee address
		static    uint8_t xbee_api_hdr[8] = {0}; //Stores the grounstation Xbee header
		static    uint8_t xbee_chksum[1] = {0};  //Stores the groundstation xbee footer

In file navigation.pde:

	List of Functions modified:
		/* Modified function to call set_orbit_parameters() & follow_orbit() in case 
		// Orbit_follower is enabled and the control mode is guided. This makes the 
		// aircraft follow an orbit around the orbit center.		
		*/		
		static void navigate()

	List of Functions added:
		None

	List of Variables added:
		None

In file ArduPlane.pde:

	List of Functions modified:
		/* If cooperative flight is enabled, this function is modified so that all 
		// interaircraft data is sent from this loop. Also, all interaircraft data
		// that is recieved and stored is used (for setting waypoint etc) from this loop.
		*/
		static void slow_loop()
	
	List of Functions added:
		None

	List of variables added:
		static struct  	Location p_guided_WP;
		static struct  	Location orbit_center;
		static struct  	Location recevied_WP;
		static int  orbit_radius;
		static float orbit_gain_factor;
		static float orbit_distance;

		#if COOPERATIVE_MISSION == ENABLED
		#include <APM_Xbee.h>//btak
		#endif


In file follower_guided_wp.pde:

/* This file contains all the implementation functions that handle the 
// interaircraft data.
*/

	List of Functions added:
		/* This is the function that sets the recevied leader location data as the 
		// guided waypoint for the follower. It does this only when its a legitimate 
		// new location packet. Also, after setting the guided waypoint it sends the 
		// particular location packet to debugg xbee(6666), as a note of confirmation.  		
		*/
		void set_leader_wp(uint8_t *wpp){ 


		/* This is the function used in case of orbit follower. A swarm of orbit followers
		// constantly exchange location information. For a circular exchange of information
		// like 1 -> 2 -> 3 -> 1, this function will decode and save the recevied location 
		// data in the struch recevied_wp for further use(like mantaining distance etc)
		*/
		void store_recevied_wp(uint8_t *wpp)

		/* This function takes the interaircraft data bytes from WPpacket, decodes the bytes to 
		// int32_t and saves them in the struct guided_WP. It modifies lat, long and alt.
		*/
		void hex_to_data(void)



		/* Sets the orbit center and shares it with other aircafts(shares only if center is set through GCS). 
		// Center could either be set using GCS guided waypoint(fly to here command) or automatically 
		// through interaircaft data packet 'evnt_WPpacket'as shared by some other aircraft
		// in the system.    
		*/
		void set_new_orbit_center(void)		

		/* Sets the orbit center, its parameters like radius, approach gain etc.
		*/
		void set_orbit_parameters(void)
	

		/* Function to set orbit following in motion. Aircraft follows an
		// orbit around the set orbit center. When far away from the center, heading is 
		// almost directed toward the center, as it approaches closer, it slips in orbit
		// as set using the orbit_gain_factor.
		*/
		void follow_orbit(void)

		/* Other utility functions
		*/		
		int check_same_packet(void)
		void copy_packet(void)
		void send_debugg_data(void)
		void copy_evnt_packet(void)
		int check_same_evnt_packet(void)
		void send_evnt_debugg_data(void)
		int check_same_GWP(void)

		
In file APM_Xbee.h:

	List of functions added
		
		/* The function that extracts the location data from the location struct 'loc', creates
		// an array of bytes that forms an API packet for xbee, and sends it to hal.uartC to be 
		// transmitted to the aircraft choosed as the first argument. The third argument tells whether
		// the information in 'loc' is for waypoint or orbit cenetr or anything else(They have different crc)
		*/
		void XBee_sendFollower(const uint8_t flwid, const struct Location *loc, const uint8_t type);

		/* For every interaircraft packet sent, a simple packet crc is calculated and sent with it
		// (inside the API packet). It serves the purpose of differentiating between different types of 
		// interaircraft packets(which have different crcs). This function adds the appropriate crc.
		*/
		void Calc_pkt_crc(uint16_t data_sum, uint8_t *crc, uint8_t type);
		void Follower_print_recieved(uint8_t *data);
		long bytes_to_int(uint8_t *hexdata);	
		void int_to_bytes(int loword, int hiword, unsigned char *hexdata);

In file APM_Config.h:
	
	/* TO enable telemetry and all interaircraft communication on 
	// UART2 (hal.uartC). Preferred this to hal.UARTA so that hal.uartA
	// could be used for running HIL simulations through USB
	*/	
	# define SERIAL3_BAUD                    57600
	# define TELEMETRY_UART2 ENABLED

	/* Disable this to prevent any interaircraft data to be sent or recieved.
	// Note mavlink_helpers.h have their own # defines to enable Xbee API communication
	*/
	 # define COOPERATIVE_MISSION ENABLED

	/* To set this aircraft as leader, it'll keep sending location data to follower
	// at all times indifferent of its control mode
	*/
	 # define AIRCRAFT_TYPE_LEADER DISABLED

	/* To set this aircraft as follower. It'll receive leader location data and followe 
	// ONLY in guided mode. Switch using RC controller or GCS. An aircraft can be leader
	// and follower simulataneously. ex LEADER(MAV1)>>----<<FOLLOWER (MAV2) LEADER >>----<<FOLLOWER(MAV3) 
	*/
	 # define AIRCRAFT_TYPE_FOLLOWER ENABLED

	/* To set this aircraft as an Orbit follower. Only enabled in guided mode, the aircraft
	// never goes to the guided waypoint but circles around it. Also, it shares the orbit center data
	// with any other aircraft in the orbit follower mode. So, set orbit center for any aircraft using 
	// GCS and all other aircrafts automatically flock to circle around that center. 		
	*/
	 # define AIRCRAFT_TYPE_ORBIT_FOLLOWER DISABLED












