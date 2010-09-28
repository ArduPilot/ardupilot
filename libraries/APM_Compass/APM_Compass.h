#ifndef APM_Compass_h
#define APM_Compass_h

#define APM_COMPASS_COMPONENTS_UP_PINS_BACK 0
#define APM_COMPONENTS_UP_PINS_BACK_LEFT 1
#define APM_COMPASS_COMPONENTS_UP_PINS_LEFT 2
#define APM_COMPONENTS_UP_PINS_FORWARD_LEFT 3
#define APM_COMPASS_COMPONENTS_UP_PINS_FORWARD 4
#define APM_COMPONENTS_UP_PINS_FORWARD_RIGHT 5
#define APM_COMPASS_COMPONENTS_UP_PINS_RIGHT 6
#define APM_COMPONENTS_UP_PINS_BACK_RIGHT 7
#define APM_COMPASS_COMPONENTS_DOWN_PINS_BACK 8
#define APM_COMPONENTS_DOWN_PINS_BACK_LEFT 9
#define APM_COMPASS_COMPONENTS_DOWN_PINS_LEFT 10
#define APM_COMPONENTS_DOWN_PINS_FORWARD_LEFT 11
#define APM_COMPASS_COMPONENTS_DOWN_PINS_FORWARD 12
#define APM_COMPONENTS_DOWN_PINS_FORWARD_RIGHT 13
#define APM_COMPASS_COMPONENTS_DOWN_PINS_RIGHT 14
#define APM_COMPONENTS_DOWN_PINS_BACK_RIGHT 15

class APM_Compass_Class
{
  private:	
	float calibration[3]; 
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
