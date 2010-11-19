#ifndef APM_ADC_h
#define APM_ADC_h

#define bit_set(p,m)   ((p) |= ( 1<<m))
#define bit_clear(p,m) ((p) &= ~(1<<m))

// We use Serial Port 2 in SPI Mode
#define ADC_DATAOUT     51    // MOSI
#define ADC_DATAIN      50    // MISO
#define ADC_SPICLOCK    52    // SCK
#define ADC_CHIP_SELECT 33    // PC4   9 // PH6  Puerto:0x08 Bit mask : 0x40

class APM_ADC_Class
{
  private:
  public:
	APM_ADC_Class();  // Constructor
	void Init();
	int Ch(unsigned char ch_num);     
};

class APM_ADC_HIL_Class
{
  private:
  public:
	APM_ADC_HIL_Class();  // Constructor
	void Init();
	int Ch(unsigned char ch_num);     
	int setHIL(float p, float q, float r, float gyroTemp,
            float aX, float aY, float aZ, float diffPress);     
};

#endif
