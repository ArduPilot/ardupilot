
#ifndef __AP_HAL_LINUX_UTIL_H__
#define __AP_HAL_LINUX_UTIL_H__

#include <AP_HAL.h>
#include "AP_HAL_Linux_Namespace.h"
#include "ToneAlarmDriver.h"
#include "DycoLEDDriver.h"

class Linux::LinuxUtil : public AP_HAL::Util {
public:
    void init(int argc, char * const *argv) {
        saved_argc = argc;
        saved_argv = argv;
    }


    bool run_debug_shell(AP_HAL::BetterStream *stream) { return false; }

    /**
       return commandline arguments, if available
     */
    void commandline_arguments(uint8_t &argc, char * const *&argv);

    bool toneAlarm_init();
    void toneAlarm_set_tune(uint8_t tune);
    void led_init(uint8_t length);
    void led_set_solid_color(uint8_t led_num, uint8_t color);
    void led_set_pattern(uint16_t led_num,uint16_t color_series[],float bright_series[],uint16_t time_series[],uint8_t res, uint8_t step_cnt);
    
    void _toneAlarm_timer_tick();
    void _led_timer_tick();

private:
    static Linux::ToneAlarm _toneAlarm;
    static Linux::LinuxDycoLEDStripDriver _ledstrip;
    int saved_argc;
    char* const *saved_argv;
};



#endif // __AP_HAL_LINUX_UTIL_H__
