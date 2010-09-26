#ifndef APM_Compass_h
#define APM_Compass_h

#include <AP_Math.h>

#define APM_COMPASS_COMPONENTS_UP_PINS_BACK 0
#define APM_COMPASS_COMPONENTS_UP_PINS_LEFT 1
#define APM_COMPASS_COMPONENTS_UP_PINS_FORWARD 2
#define APM_COMPASS_COMPONENTS_UP_PINS_RIGHT 3
#define APM_COMPASS_COMPONENTS_DOWN_PINS_BACK 4
#define APM_COMPASS_COMPONENTS_DOWN_PINS_LEFT 5
#define APM_COMPASS_COMPONENTS_DOWN_PINS_FORWARD 6
#define APM_COMPASS_COMPONENTS_DOWN_PINS_RIGHT 7

class APM_Compass_Class
{
  private:	
  public:
	int Mag_X;
	int Mag_Y;
	int Mag_Z;
	float Heading;
	float Heading_X;
	float Heading_Y;
	int orientation;
	
	APM_Compass_Class();  // Constructor
	void Init();
	void Read();
	void Calculate(float roll, float pitch);
	void SetOrientation(int newOrientation);
};

extern APM_Compass_Class APM_Compass;

#endif