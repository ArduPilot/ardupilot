#ifndef Waypoints_h
#define Waypoints_h

#include <inttypes.h>
#include <avr/eeprom.h>
#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

class Waypoints
{
  public:
	Waypoints();

	struct WP {
		uint8_t id;					// for commands
		int8_t p1;					// for commands
		int32_t alt;				// Altitude in centimeters (meters * 100)
		int32_t lat;				// Lattitude * 10**7
		int32_t lng;				// Longitude * 10**7
	};
	
	WP			get_waypoint_with_index(uint8_t i);
	WP			get_current_waypoint(void);

	void 		set_waypoint_with_index(Waypoints::WP wp, uint8_t i);

	void 		set_start_byte(uint16_t start_byte);
	void 		set_wp_size(uint8_t wp_size);
	
	void		next_index(void);
	uint8_t 	get_index(void);
	void 		set_index(uint8_t i);

	uint8_t 	get_total(void);
	void 		set_total(uint8_t total);
	


  private:
	uint16_t _start_byte;
	uint8_t _wp_size;
	uint8_t _index;
	uint8_t _total;	
};

#endif

