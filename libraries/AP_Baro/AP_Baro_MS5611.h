#ifndef APM_MS5611_h
#define APM_MS5611_h

#define MS5611_CS A2 // Chip select pin  (provisional)

#define CMD_MS5611_RESET 0x1E
#define CMD_MS5611_PROM_Setup 0xA0
#define CMD_MS5611_PROM_C1 0xA2
#define CMD_MS5611_PROM_C2 0xA4
#define CMD_MS5611_PROM_C3 0xA6
#define CMD_MS5611_PROM_C4 0xA8
#define CMD_MS5611_PROM_C5 0xAA
#define CMD_MS5611_PROM_C6 0xAC
#define CMD_MS5611_PROM_CRC 0xAE
#define CMD_CONVERT_D1_OSR4096 0x48   // Maximun resolution
#define CMD_CONVERT_D2_OSR4096 0x58   // Maximun resolution


class APM_MS5611_Class
{
  private:
	// Internal calibration registers
    uint16_t C1,C2,C3,C4,C5,C6;
	uint32_t D1,D2;
	void calculate();
    uint8_t MS5611_Ready();
	long MS5611_timer;
    uint8_t MS5611_State;

  public:
  //uint16_t C1,C2,C3,C4,C5,C6;
  //uint32_t D1,D2;
	int16_t Temp;
	int32_t Press;
	int32_t Alt;


	APM_MS5611_Class();  // Constructor
	void init();
	uint8_t read();
	uint32_t get_pressure();     // in mbar*100 units
	uint16_t get_temperature();  // in celsius degrees * 100 units
	float get_altitude();        // in meter units
};

#endif
