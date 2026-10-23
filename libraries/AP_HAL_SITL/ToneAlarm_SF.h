#pragma once

#include "AP_HAL_SITL.h"

namespace HALSITL {
    class ToneAlarm_SF {
    public:
        void set_buzzer_tone(float frequency, float volume, float duration_ms);
        bool init();
    };
}
