#ifndef GPS_NMEA_h
#define GPS_NMEA_h

#include <inttypes.h>

#define GPS_BUFFERSIZE 120 

class GPS_NMEA_Class
{
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

  public:
    // Methods
	GPS_NMEA_Class();
	void Init();
	void Read();
	// Properties
	long Time;          //GPS Millisecond Time of Week
	long Lattitude;     // Geographic coordinates
	long Longitude;
	long Altitude;
	long Ground_Speed;
	long Speed_3d;      //Speed (3-D)
	long Ground_Course;
	uint8_t Type;       // Type of GPS (library used)
	uint8_t NumSats;    // Number of visible satelites
	uint8_t Fix;        // >=1:GPS FIX   0:No FIX (normal logic)
	uint8_t Quality;    // GPS Signal quality
	uint8_t NewData;    // 1:New GPS Data
	uint8_t PrintErrors; // 1: To Print GPS Errors (for debug)
	int HDOP;            // HDOP
};

extern GPS_NMEA_Class GPS;

#endif