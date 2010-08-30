#ifndef AP_GPS_UBLOX_h
#define AP_GPS_UBLOX_h

#include <GPS.h>
#define MAXPAYLOAD 100

class AP_GPS_406 : public GPS
{
  public:
    // Methods
	AP_GPS_406();
	void init();
	void update();

  private:
    // Internal variables
	uint8_t step;
	uint8_t payload_counter;
	static uint8_t buffer[MAXPAYLOAD];
	
	void parse_gps();
	int32_t join_4_bytes(uint8_t Buffer[]);
	int16_t join_2_bytes(uint8_t Buffer[]);
	void change_to_sirf_protocol(void);
	void configure_gps(void);
};

#endif

