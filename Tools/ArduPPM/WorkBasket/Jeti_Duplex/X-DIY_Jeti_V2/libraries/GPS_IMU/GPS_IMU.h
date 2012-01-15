#ifndef GPS_IMU_h
#define GPS_IMU_h

#include <inttypes.h>

#define IMU_MAXPAYLOAD 32

class GPS_IMU_Class
{
  private:
    // Internal variables
	union int_union {
		int16_t word;
		uint8_t	byte[2];
	} intUnion;

	union long_union {
		int32_t dword;
		uint8_t	byte[4];
	} longUnion;

	uint8_t ck_a;     // Packet checksum
	uint8_t ck_b;
	uint8_t IMU_ck_a;
	uint8_t IMU_ck_b;
	uint8_t IMU_step;
	uint8_t IMU_class;
	uint8_t message_num;
	uint8_t payload_length;
	uint8_t payload_counter;
	uint8_t buffer[IMU_MAXPAYLOAD];
	
	long IMU_timer;
	long IMU_ecefVZ;
	void IMU_join_data();
	void IMU2_join_data();
	void GPS_join_data();
	void checksum(unsigned char IMU_data);
	long join_4_bytes(unsigned char Buffer[]);

  public:
    // Methods
	GPS_IMU_Class();
	void Init();
	void Read();
	
	// Properties
	long 	roll_sensor;		// how much we're turning in deg * 100
	long 	pitch_sensor;		// our angle of attack in deg * 100
	int 	airspeed;
	float 	imu_health;
	uint8_t imu_ok;
	
	long Time;          //GPS Millisecond Time of Week
	long Lattitude;     // Geographic coordinates
	long Longitude;
	long Altitude;
	long Ground_Speed;
	long Ground_Course;
	long Speed_3d;
	
	uint8_t NumSats;      // Number of visible satelites
	uint8_t Fix;        // 1:GPS FIX   0:No FIX (normal logic)
	uint8_t NewData;    // 1:New GPS Data
	uint8_t PrintErrors; // 1: To Print GPS Errors (for debug)
};

extern GPS_IMU_Class GPS;
#endif