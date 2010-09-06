// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
#ifndef AP_GPS_UBLOX_h
#define AP_GPS_UBLOX_h

#include <GPS.h>
#define MAXPAYLOAD 60

class AP_GPS_UBLOX : public GPS
{
  public:
    // Methods
	AP_GPS_UBLOX(Stream *s);
	void init();
	void update();

  private:
    // Internal variables
	uint8_t ck_a;     // Packet checksum
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
	long ecefVZ;
	void parse_gps();
	void checksum(unsigned char data);
	long join_4_bytes(unsigned char Buffer[]);
};

#endif
