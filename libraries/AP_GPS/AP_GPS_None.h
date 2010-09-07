#ifndef AP_GPS_None_h
#define AP_GPS_None_h

#include <GPS.h>

class AP_GPS_None : public GPS
{
  public:
	AP_GPS_None();
	void init();
	void update();

  private:

};
#endif
