#ifndef AP_Compass_HIL_H
#define AP_Compass_HIL_H

#include <Compass.h>
#include "AP_Compass_HMC5843.h" // to get #defines since we are modelling this
#include "WProgram.h"

class AP_Compass_HIL : public Compass
{
  public:
	AP_Compass_HIL();  // Constructor
	
	bool 		init(int initialise_wire_lib = 1);
	void 		read();
	void 		calculate(float roll, float pitch);
	void 		set_orientation(const Matrix3f &rotation_matrix);
	void 		set_offsets(int x, int y, int z);
	void 		set_declination(float radians);
    void 		setHIL(float Mag_X, float Mag_Y, float Mag_Z);

  private:
	int 		orientation;
	Matrix3f 	orientation_matrix;
	float 		calibration[3];
	int 		offset[3];
	float 		declination;
};

#endif
