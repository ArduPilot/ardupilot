#pragma once

#include "AP_HAL_Linux.h"
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO
#include <AP_HAL_Linux/RCOutput_Bebop.h>
#include "ToneAlarm.h"

namespace Linux {

#define TONEALARM_PWM_POWER 20

class ToneAlarm_Disco : public ToneAlarm {
public:
    ToneAlarm_Disco();
    bool init() override;
    void set_buzzer_tone(float frequency, float volume, uint32_t duration_ms) override;

private:
    RCOutput_Bebop *bebop_out;
};

}
#endif
