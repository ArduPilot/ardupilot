
#ifndef __AP_BARO_BMP085_HIL_H__
#define __AP_BARO_BMP085_HIL_H__

#include "AP_Baro.h"

class AP_Baro_BMP085_HIL
{
  private:
    uint8_t BMP085_State;
  public:
    AP_Baro_BMP085_HIL();  // Constructor
	int32_t RawPress;
	int32_t RawTemp;
	int16_t Temp;
	int32_t Press;
	//int Altitude;
	uint8_t oss;
	void Init(int initialiseWireLib = 1, bool apm2_hardware=false);
	uint8_t Read();
    void setHIL(float Temp, float Press);
    int32_t _offset_press;
};

#endif //  __AP_BARO_BMP085_HIL_H__
