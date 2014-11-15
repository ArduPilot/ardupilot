
#ifndef __AP_HAL_LINUX_UTIL_H__
#define __AP_HAL_LINUX_UTIL_H__

#include <AP_HAL.h>
#include "AP_HAL_Linux_Namespace.h"
#include "ToneAlarmDriver.h"

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
    
    void _toneAlarm_timer_tick();

private:
	static Linux::ToneAlarm _toneAlarm;
    int saved_argc;
    char* const *saved_argv;
};



#endif // __AP_HAL_LINUX_UTIL_H__
