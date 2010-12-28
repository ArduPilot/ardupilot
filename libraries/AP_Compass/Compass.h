#ifndef Compass_h
#define Compass_h

#include <inttypes.h>
#include "../AP_Math/AP_Math.h"

class Compass
{
  public:
	int mag_x;
	int mag_y;
	int mag_z;
	float heading;
	float heading_x;
	float heading_y;
	unsigned long last_update;	
	
	virtual bool init(int initialise_wire_lib = 1) = 0;
	virtual void read() = 0;
	virtual void calculate(float roll, float pitch) = 0;
	virtual void set_orientation(const Matrix3f &rotation_matrix) = 0;
	virtual void set_offsets(int x, int y, int z) = 0;
	virtual void set_declination(float radians) = 0;
};
#endif
