#ifndef APM_Compass_h
#define APM_Compass_h

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
	
	APM_Compass_Class();  // Constructor
	void Init();
	void Read();
	void Calculate(float roll, float pitch);
};

extern APM_Compass_Class APM_Compass;

#endif