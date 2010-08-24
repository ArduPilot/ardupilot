#ifndef AP_GPS_h
#define AP_GPS_h

#include <inttypes.h>
#include "WProgram.h"

class AP_GPS
{
	public:
	// Methods
	virtual void init();
	virtual void update();

	// Properties
	long time;					//GPS Millisecond Time of Week
	long lattitude;		 // Geographic coordinates
	long longitude;
	long altitude;
	long ground_speed;
	long ground_course;
	long speed_3d;
	uint8_t num_sats;			// Number of visible satelites
	uint8_t fix;				// 1:GPS FIX	 0:No FIX (normal logic)
	uint8_t new_data;		// 1:New GPS Data
	uint8_t print_errors; // 1: To Print GPS Errors (for debug)
	long GPS_timer;

	union long_union {
		int32_t dword;
		uint8_t	byte[4];
	} longUnion;

	union int_union {
		int16_t word;
		uint8_t	byte[2];
	} intUnion;
};

#endif
