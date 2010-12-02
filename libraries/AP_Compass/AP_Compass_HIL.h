#ifndef AP_Compass_HIL_H
#define AP_Compass_HIL_H

#include <Compass.h>
#include "AP_Compass_HMC5843.h" // to get #defines since we are modelling this
#include "WProgram.h"

class AP_Compass_HIL : public Compass
{
  private:
	int orientation;
	Matrix3f orientationMatrix;
	float calibration[3];
	int offset[3];
	float declination;
  public:
	AP_Compass_HIL();  // Constructor
	bool init(int initialiseWireLib = 1);
	void read();
	void calculate(float roll, float pitch);
	void setOrientation(const Matrix3f &rotationMatrix);
	void setOffsets(int x, int y, int z);
	void setDeclination(float radians);
    void setHIL(float Mag_X, float Mag_Y, float Mag_Z);
};

#endif
