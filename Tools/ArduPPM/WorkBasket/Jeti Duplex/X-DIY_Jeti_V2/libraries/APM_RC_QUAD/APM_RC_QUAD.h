#ifndef APM_RC_QUAD_h
#define APM_RC_QUAD_h

#define NUM_CHANNELS 8
#define MIN_PULSEWIDTH 900
#define MAX_PULSEWIDTH 2100

class APM_RC_QUAD_Class
{
  private:
  public:
	APM_RC_QUAD_Class();
	void Init();
	void OutputCh(unsigned char ch, int pwm);
	int InputCh(unsigned char ch);
	unsigned char GetState();
};

extern APM_RC_QUAD_Class APM_RC_QUAD;

#endif