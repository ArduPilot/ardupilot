#ifndef AP_Compass_h
#define AP_Compass_h

#include <Compass.h>

class AP_Compass : public Compass
{
  public:
	AP_Compass();  // Constructor
	void init();
	void update();
	void calculate(float roll, float pitch);
	
  private:	
};

#endif