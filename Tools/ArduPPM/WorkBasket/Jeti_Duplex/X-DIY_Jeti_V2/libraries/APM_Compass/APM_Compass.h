#ifndef APM_Compass_h
#define APM_Compass_h

#include "../AP_Math/AP_Math.h"

// Rotation matrices
#define ROTATION_NONE				Matrix3f(1, 0, 0, 0, 1, 0, 0 ,0, 1)
#define ROTATION_YAW_45				Matrix3f(0.70710678, -0.70710678, 0, 0.70710678, 0.70710678, 0, 0, 0, 1)
#define ROTATION_YAW_90				Matrix3f(0, -1, 0, 1, 0, 0, 0, 0, 1)
#define ROTATION_YAW_135			Matrix3f(-0.70710678, -0.70710678, 0, 0.70710678, -0.70710678, 0, 0, 0, 1)
#define ROTATION_YAW_180			Matrix3f(-1, 0, 0, 0, -1, 0, 0, 0, 1)
#define ROTATION_YAW_225			Matrix3f(-0.70710678, 0.70710678, 0, -0.70710678, -0.70710678, 0, 0, 0, 1)
#define ROTATION_YAW_270			Matrix3f(0, 1, 0, -1, 0, 0, 0, 0, 1)
#define ROTATION_YAW_315			Matrix3f(0.70710678, 0.70710678, 0, -0.70710678, 0.70710678, 0, 0, 0, 1)
#define ROTATION_ROLL_180 			Matrix3f(1, 0, 0, 0, -1, 0, 0, 0, -1)
#define ROTATION_ROLL_180_YAW_45	Matrix3f(0.70710678, 0.70710678, 0, 0.70710678, -0.70710678, 0, 0, 0, -1)
#define ROTATION_ROLL_180_YAW_90	Matrix3f(0, 1, 0, 1, 0, 0, 0, 0, -1)
#define ROTATION_ROLL_180_YAW_135	Matrix3f(-0.70710678, 0.70710678, 0, 0.70710678, 0.70710678, 0, 0, 0, -1)
#define ROTATION_PITCH_180			Matrix3f(-1, 0, 0, 0, 1, 0, 0, 0, -1)
#define ROTATION_ROLL_180_YAW_225	Matrix3f(-0.70710678, -0.70710678, 0, -0.70710678, 0.70710678, 0, 0, 0, -1)
#define ROTATION_ROLL_180_YAW_270	Matrix3f(0, -1, 0, -1, 0, 0, 0, 0, -1)
#define ROTATION_ROLL_180_YAW_315	Matrix3f(0.70710678, -0.70710678, 0, -0.70710678, -0.70710678, 0, 0, 0, -1)

#define APM_COMPASS_COMPONENTS_UP_PINS_FORWARD ROTATION_NONE
#define APM_COMPONENTS_UP_PINS_FORWARD_RIGHT ROTATION_YAW_45
#define APM_COMPASS_COMPONENTS_UP_PINS_RIGHT ROTATION_YAW_90
#define APM_COMPONENTS_UP_PINS_BACK_RIGHT ROTATION_YAW_135
#define APM_COMPASS_COMPONENTS_UP_PINS_BACK ROTATION_YAW_180
#define APM_COMPONENTS_UP_PINS_BACK_LEFT ROTATION_YAW_225
#define APM_COMPASS_COMPONENTS_UP_PINS_LEFT ROTATION_YAW_270
#define APM_COMPONENTS_UP_PINS_FORWARD_LEFT ROTATION_YAW_315
#define APM_COMPASS_COMPONENTS_DOWN_PINS_FORWARD ROTATION_ROLL_180
#define APM_COMPONENTS_DOWN_PINS_FORWARD_RIGHT ROTATION_ROLL_180_YAW_45
#define APM_COMPASS_COMPONENTS_DOWN_PINS_RIGHT ROTATION_ROLL_180_YAW_90
#define APM_COMPONENTS_DOWN_PINS_BACK_RIGHT ROTATION_ROLL_180_YAW_135
#define APM_COMPASS_COMPONENTS_DOWN_PINS_BACK ROTATION_PITCH_180
#define APM_COMPONENTS_DOWN_PINS_BACK_LEFT ROTATION_ROLL_180_YAW_225
#define APM_COMPASS_COMPONENTS_DOWN_PINS_LEFT ROTATION_ROLL_180_YAW_270
#define APM_COMPONENTS_DOWN_PINS_FORWARD_LEFT ROTATION_ROLL_180_YAW_315

class APM_Compass_Class
{
  private:
	int orientation;
	Matrix3f orientationMatrix;
	float calibration[3];
	int offset[3];
	float declination;
  public:
	int Mag_X;
	int Mag_Y;
	int Mag_Z;
	float Heading;
	float Heading_X;
	float Heading_Y;
	unsigned long lastUpdate;	
	
	APM_Compass_Class();  // Constructor
	bool Init();
	void Read();
	void Calculate(float roll, float pitch);
	void SetOrientation(const Matrix3f &rotationMatrix);
	void SetOffsets(int x, int y, int z);
	void SetDeclination(float radians);
};

extern APM_Compass_Class APM_Compass;

#endif
