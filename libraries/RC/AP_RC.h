#ifndef AP_RC_h
#define AP_RC_h

#include <inttypes.h>
#include "WProgram.h"
#include "RC.h"

class AP_RC : public RC
{
  public:
	AP_RC();
	void read_pwm();
	void set_ch_pwm(uint8_t ch, uint16_t pwm);
	void init();
	void set_throttle(float percent);
	void trim();
	
	int16_t radio_in[4];
	int16_t radio_min[4];
	int16_t radio_trim[4];
	int16_t radio_max[4];
	
	float servo_out[4];

  private:
	uint16_t _timer_out;
};

#endif

