#ifndef APM_Compass_h
#define APM_Compass_h

#define APM_COMPASS_COMPONENTS_UP_PINS_FORWARD 0
#define APM_COMPONENTS_UP_PINS_FORWARD_RIGHT 1
#define APM_COMPASS_COMPONENTS_UP_PINS_RIGHT 2
#define APM_COMPONENTS_UP_PINS_BACK_RIGHT 3
#define APM_COMPASS_COMPONENTS_UP_PINS_BACK 4
#define APM_COMPONENTS_UP_PINS_BACK_LEFT 5
#define APM_COMPASS_COMPONENTS_UP_PINS_LEFT 6
#define APM_COMPONENTS_UP_PINS_FORWARD_LEFT 7
#define APM_COMPASS_COMPONENTS_DOWN_PINS_FORWARD 8
#define APM_COMPONENTS_DOWN_PINS_FORWARD_RIGHT 9
#define APM_COMPASS_COMPONENTS_DOWN_PINS_RIGHT 10
#define APM_COMPONENTS_DOWN_PINS_BACK_RIGHT 11
#define APM_COMPASS_COMPONENTS_DOWN_PINS_BACK 12
#define APM_COMPONENTS_DOWN_PINS_BACK_LEFT 13
#define APM_COMPASS_COMPONENTS_DOWN_PINS_LEFT 14
#define APM_COMPONENTS_DOWN_PINS_FORWARD_LEFT 15

class APM_Compass_Class
{
  private:
	int orientation;
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
	void Init();
	void Read();
	void Calculate(float roll, float pitch);
	void SetOrientation(int newOrientation);
	void SetOffsets(int x, int y, int z);
	void SetDeclination(float radians);
};

extern APM_Compass_Class APM_Compass;

#endif
