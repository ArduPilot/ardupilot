#ifndef APM_RC_h
#define APM_RC_h

#define NUM_CHANNELS 8
#define MIN_PULSEWIDTH 900
#define MAX_PULSEWIDTH 2100

class APM_RC_Class
{
  private:
  public:
	APM_RC_Class();
	void Init();
	void OutputCh(unsigned char ch, int pwm);
	int InputCh(unsigned char ch);
	unsigned char GetState();
	void Force_Out0_Out1(void);
	void Force_Out2_Out3(void);
	void Force_Out6_Out7(void);
};

extern APM_RC_Class APM_RC;

#endif