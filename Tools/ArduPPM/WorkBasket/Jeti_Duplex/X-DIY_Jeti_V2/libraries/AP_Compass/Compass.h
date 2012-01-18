#ifndef Compass_h
#define Compass_h

#include <inttypes.h>

class Compass
{
  public:
	virtual void init();
	virtual void update();
	virtual void calculate(float roll, float pitch);

	int16_t mag_X;
	int16_t mag_Y;
	int16_t mag_Z;
	int32_t ground_course;
	float heading;
	float heading_X;
	float heading_Y;

};

#endif