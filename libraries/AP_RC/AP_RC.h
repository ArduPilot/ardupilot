#ifndef AP_RC_h
#define AP_RC_h

#include <inttypes.h>

class AP_RC
{
  private:
  public:
	AP_RC();
	void 		init();	
	void 		output_ch_pwm(uint8_t ch, uint16_t pwm);
	uint16_t 	input_ch(uint8_t ch);
};

#endif
