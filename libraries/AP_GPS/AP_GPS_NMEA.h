#ifndef AP_GPS_NMEA_h
#define AP_GPS_NMEA_h

#include <GPS.h>
#define GPS_BUFFERSIZE 120 

class AP_GPS_NMEA : public GPS
{
  public:
    // Methods
	AP_GPS_NMEA();
	void init();
	void update();

	// Properties
	uint8_t quality;    // GPS Signal quality
	int HDOP;            // HDOP

  private:
    // Internal variables
    uint8_t GPS_checksum;
    uint8_t GPS_checksum_calc;
	char buffer[GPS_BUFFERSIZE];
	int bufferidx;

	void parse_nmea_gps(void);
	uint8_t parseHex(char c);
	long parsedecimal(char *str,uint8_t num_car);
	long parsenumber(char *str,uint8_t numdec);

};

#endif