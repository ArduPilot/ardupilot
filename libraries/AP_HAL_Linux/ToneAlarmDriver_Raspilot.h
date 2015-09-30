#ifndef __TONE_ALARM_DRIVER_RASPILOT_H__
#define __TONE_ALARM_DRIVER_RASPILOT_H__

#include "AP_HAL_Linux.h"

#include "ToneAlarmDriver.h"

class Linux::ToneAlarm_Raspilot : public Linux::ToneAlarm {
public:
  ToneAlarm_Raspilot();
  bool init();
	void stop();
	bool play();

private:
  int  mem_fd;
  void *gpio_map;
  void *pwm_map;
  void *clk_map;
  volatile uint32_t *gpio;
  volatile uint32_t *pwm;
  volatile uint32_t *clk;

  void setPWM0Period(uint32_t time_us);
  void setPWM0Duty(uint8_t percent);
};

#endif
