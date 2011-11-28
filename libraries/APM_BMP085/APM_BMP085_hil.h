#ifndef APM_BMP085_hil_h
#define APM_BMP085_hil_h

class APM_BMP085_HIL_Class
{
  private:
    uint8_t BMP085_State;
  public:
  	APM_BMP085_HIL_Class();  // Constructor
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

#endif
