// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
#ifndef AP_GPS_NMEA_h
#define AP_GPS_NMEA_h

#include <GPS.h>
#define GPS_BUFFERSIZE 120 

#define NMEA_OUTPUT_SENTENCES 	"$PMTK314,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n" //Set GPGGA and GPVTG

#define NMEA_BAUD_RATE_4800    	"$PSRF100,1,4800,8,1,0*0E\r\n"
#define NMEA_BAUD_RATE_9600    	"$PSRF100,1,9600,8,1,0*0D\r\n"
#define NMEA_BAUD_RATE_19200    "$PSRF100,1,19200,8,1,0*38\r\n"
#define NMEA_BAUD_RATE_38400    "$PSRF100,1,38400,8,1,0*3D\r\n"  
#define NMEA_BAUD_RATE_57600    "$PSRF100,1,57600,8,1,0*36\r\n"

#define NMEA_OUTPUT_1HZ		"$PMTK220,1000*1F\r\n"
#define NMEA_OUTPUT_2HZ		"$PMTK220,500*2B\r\n"
#define NMEA_OUTPUT_4HZ		"$PMTK220,250*29\r\n"
#define NMEA_OTUPUT_5HZ		"$PMTK220,200*2C\r\n"
#define NMEA_OUTPUT_10HZ	"$PMTK220,100*2F\r\n"

#define SBAS_ON			"$PMTK313,1*2E\r\n"
#define SBAS_OFF		"$PMTK313,0*2F\r\n"

#define WAAS_ON			"$PSRF151,1*3F\r\n"
#define WAAS_OFF		"$PSRF151,0*3E\r\n"

#define DGPS_OFF		"$PMTK301,0*2C\r\n"
#define DGPS_RTCM		"$PMTK301,1*2D\r\n"
#define DGPS_SBAS		"$PMTK301,2*2E\r\n"

#define DATUM_GOOGLE		"$PMTK330,0*2E\r\n"

class AP_GPS_NMEA : public GPS
{
  public:
    // Methods
	AP_GPS_NMEA(Stream *s);
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
