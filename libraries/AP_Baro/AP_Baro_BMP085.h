/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef __AP_BARO_BMP085_H__
#define __AP_BARO_BMP085_H__

#define TEMP_FILTER_SIZE 4
#define PRESS_FILTER_SIZE 8

#include "AP_Baro.h"

class AP_Baro_BMP085 : public AP_Baro
{
  public:
	AP_Baro_BMP085(bool apm2_hardware):
			_temp_index(0),
			_press_index(0),
            _apm2_hardware(apm2_hardware){};  // Constructor


    /* AP_Baro public interface: */
    bool    init(AP_PeriodicProcess * scheduler);
    uint8_t read();
    int32_t get_pressure();
    int16_t get_temperature();
    float   get_altitude();

    int32_t get_raw_pressure();
    int32_t get_raw_temp();

  private:
	int32_t RawPress;
	int32_t	_offset_press;
	int32_t RawTemp;
	int16_t Temp;
	int32_t Press;
	//int Altitude;
	uint8_t oss;
	bool _apm2_hardware;
	//int32_t Press0;  // Pressure at sea level


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
    uint32_t _retry_time;

	void Command_ReadPress();
	void Command_ReadTemp();
	void ReadPress();
	void ReadTemp();
	void Calculate();
};

#endif // __AP_BARO_BMP085_H__
