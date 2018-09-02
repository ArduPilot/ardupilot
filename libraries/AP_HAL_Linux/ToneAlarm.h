#pragma once

#include <AP_HAL/AP_HAL.h>

namespace Linux {

class ToneAlarm {
public:
	ToneAlarm();
	virtual bool init();
    virtual void set_buzzer_tone(float frequency, float volume, uint32_t duration_ms);

private:
	int32_t period_fd;
	int32_t duty_fd;
	int32_t run_fd;
};

}
