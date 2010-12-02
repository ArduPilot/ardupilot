#ifndef Compass_h
#define Compass_h

#include <inttypes.h>
#include <AP_Math.h>

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
	
	virtual bool init(int initialise_wire_lib = 1);
	virtual void read();
	virtual void calculate(float roll, float pitch);
	virtual void set_orientation(const Matrix3f &rotation_matrix);
	virtual void set_offsets(int x, int y, int z);
	virtual void set_declination(float radians);
};
#endif
