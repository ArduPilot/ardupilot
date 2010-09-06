// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
#ifndef AP_GPS_MTK_h
#define AP_GPS_MTK_h

#include <GPS.h>
#define MAXPAYLOAD 32

#define MTK_SET_BINARY	"$PGCMD,16,0,0,0,0,0*6A\r\n"
#define MTK_SET_NMEA	"$PGCMD,16,1,1,1,1,1*6B\r\n"

#define MTK_OUTPUT_1HZ	"$PMTK220,1000*1F\r\n"
#define MTK_OUTPUT_2HZ	"$PMTK220,500*2B\r\n"
#define MTK_OUTPUT_4HZ	"$PMTK220,250*29\r\n"
#define MTK_OTUPUT_5HZ	"$PMTK220,200*2C\r\n"
#define MTK_OUTPUT_10HZ	"$PMTK220,100*2F\r\n"

#define MTK_BAUD_RATE_38400 "$PMTK251,38400*27\r\n"

#define SBAS_ON			"$PMTK313,1*2E\r\n"
#define SBAS_OFF		"$PMTK313,0*2F\r\n"

#define WAAS_ON			"$PSRF151,1*3F\r\n"
#define WAAS_OFF		"$PSRF151,0*3E\r\n"

class AP_GPS_MTK : public GPS
{
  public:
    // Methods
	AP_GPS_MTK(Stream *s);
	void init();
	void update();

  private:
	// Packet checksums
	uint8_t ck_a;     
	uint8_t ck_b;
	uint8_t GPS_ck_a;
	uint8_t GPS_ck_b;

	uint8_t step;
	uint8_t msg_class;
	uint8_t id;
	uint8_t payload_length_hi;
	uint8_t payload_length_lo;
	uint8_t payload_counter;
	uint8_t buffer[MAXPAYLOAD];

	void parse_gps();
	void checksum(unsigned char data);
};
#endif
