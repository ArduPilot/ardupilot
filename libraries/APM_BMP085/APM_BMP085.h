#ifndef APM_BMP085_h
#define APM_BMP085_h

#define TEMP_FILTER_SIZE 2
#define PRESS_FILTER_SIZE 2

#include "APM_BMP085_hil.h"

class APM_BMP085_Class
{
  public:
	APM_BMP085_Class():
			_temp_index(0),
			_press_index(0){};  // Constructor
	int32_t RawPress;
	int32_t	_offset_press;
	int32_t RawTemp;
	int16_t Temp;
	int32_t Press;
	//int Altitude;
	uint8_t oss;
	//int32_t Press0;  // Pressure at sea level

	void Init(int initialiseWireLib = 1);
	uint8_t Read();

  private:
    // State machine
    uint8_t BMP085_State;
	// Internal calibration registers
	int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
    uint16_t ac4, ac5, ac6;

	int	 	_temp_filter[TEMP_FILTER_SIZE];
	int	 	_press_filter[PRESS_FILTER_SIZE];
	long	_offset_temp;

	uint8_t	_temp_index;
	uint8_t	_press_index;

	void Command_ReadPress();
	void Command_ReadTemp();
	void ReadPress();
	void ReadTemp();
	void Calculate();
};

#endif
