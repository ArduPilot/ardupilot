#ifndef AP_RC_h
#define AP_RC_h

#include <inttypes.h>
#include "WProgram.h"

class AP_RC 
{
  public:
	AP_RC();
	void set_ch_pwm(uint8_t ch, uint16_t pwm);
	void init(int trims[]);
	void read_pwm();
	
	uint16_t input[4];
	
  private:
	uint16_t _timer_out;
};

#endif

