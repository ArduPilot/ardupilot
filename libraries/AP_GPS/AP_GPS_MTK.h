#ifndef AP_GPS_MTK_h
#define AP_GPS_MTK_h

#include <AP_GPS.h>
#define MAXPAYLOAD 32

class AP_GPS_MTK : public AP_GPS
{
  public:
    // Methods
	AP_GPS_MTK();
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
	long join_4_bytes(unsigned char Buffer[]);
};
#endif
