#ifndef Compass_h
#define Compass_h

#include <inttypes.h>
#include <AP_Math.h>

class Compass
{
  public:
	int magX;
	int magY;
	int magZ;
	float heading;
	float headingX;
	float headingY;
	unsigned long lastUpdate;	
	
	// 
	virtual bool init(int initialiseWireLib = 1);
	virtual void read();
	virtual void calculate(float roll, float pitch);
	virtual void setOrientation(const Matrix3f &rotationMatrix);
	virtual void setOffsets(int x, int y, int z);
	virtual void setDeclination(float radians);
};
#endif
