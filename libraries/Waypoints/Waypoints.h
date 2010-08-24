#ifndef Waypoints_h
#define Waypoints_h

#include <inttypes.h>
#include "WProgram.h"
#include <avr/eeprom.h>

class Waypoints
{
  public:
	Waypoints(uint16_t start_byte, uint8_t wp_size, uint8_t total);

	struct WP {
		uint8_t id;					// for commands
		int8_t p1;					// for commands
		int32_t alt;				// Altitude in centimeters (meters * 100)
		int32_t lat;				// Lattitude * 10**7
		int32_t lng;				// Longitude * 10**7
	};


	Waypoints::WP get_waypoint_with_index(uint16_t i);
	Waypoints::WP get_next_waypoint(void);
	
	void 		set_waypoint_with_index(Waypoints::WP wp, uint16_t i);
	uint8_t 	get_index(void);
	void 		set_index(uint8_t i);

	uint8_t 	get_total(void);
	


  private:
	uint16_t _start_byte;
	uint8_t _wp_size;
	uint8_t _index;
	uint8_t _total;	
};

#endif

